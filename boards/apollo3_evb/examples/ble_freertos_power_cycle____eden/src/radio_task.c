//*****************************************************************************
//
//! @file radio_task.c
//!
//! @brief Task to handle radio operation.
//!
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.5.1 of the AmbiqSuite Development Package.
//
//*****************************************************************************

//*****************************************************************************
//
// Global includes for this project.
//
//*****************************************************************************
#include "ble_freertos_power_cycle.h"

//*****************************************************************************
//
// WSF standard includes.
//
//*****************************************************************************
#include "wsf_types.h"
#include "wsf_trace.h"
#include "wsf_buf.h"
#include "wsf_timer.h"

//*****************************************************************************
//
// Includes for operating the ExactLE stack.
//
//*****************************************************************************
#include "hci_handler.h"
#include "dm_handler.h"
#include "l2c_handler.h"
#include "att_handler.h"
#include "smp_handler.h"
#include "l2c_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_api.h"
#include "hci_core.h"
#include "hci_drv.h"
#include "hci_drv_apollo.h"
#include "hci_drv_apollo3.h"
#include "hci_apollo_config.h"

#include "wsf_msg.h"

//*****************************************************************************
//
//
//*****************************************************************************
#include "tag_api.h"
#include "app_ui.h"

//*****************************************************************************
//
// Radio task handle.
//
//*****************************************************************************
TaskHandle_t radio_task_handle;

//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************
void exactle_stack_init(void);
void button_handler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);
void setup_buttons(void);

//*****************************************************************************
//
// Timer for buttons.
//
//*****************************************************************************
wsfHandlerId_t ButtonHandlerId;
wsfTimer_t ButtonTimer;

bool ble_on = false;

#define BUTTON_TIMER_EVENT 0xA0
//*****************************************************************************
//
// Timer for power cycle BLE.
//
//*****************************************************************************

wsfTimer_t PowerCycleTimer;
#define POWERCYCLE_TIMER_EVENT 0xA1

//*****************************************************************************
//
// WSF buffer pools.
//
//*****************************************************************************
#define WSF_BUF_POOLS               4

// Important note: the size of g_pui32BufMem should includes both overhead of internal
// buffer management structure, wsfBufPool_t (up to 16 bytes for each pool), and pool
// description (e.g. g_psPoolDescriptors below).

// Memory for the buffer pool
// extra AMOTA_PACKET_SIZE bytes for OTA handling
static uint32_t g_pui32BufMem[
        (WSF_BUF_POOLS*16
         + 16*8 + 32*4 + 64*6 + 280*8) / sizeof(uint32_t)];

// Default pool descriptor.
static wsfBufPoolDesc_t g_psPoolDescriptors[WSF_BUF_POOLS] =
{
    {  16,  8 },
    {  32,  4 },
    {  64,  6 },
    { 280,  8 }
};

//*****************************************************************************
//
// Tracking variable for the scheduler timer.
//
//*****************************************************************************

void radio_timer_handler(void);



//*****************************************************************************
//
// Poll the buttons.
//
//*****************************************************************************
void
button_handler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    if ( pMsg->event == BUTTON_TIMER_EVENT )
    {
        //
        // Restart the button timer.
        //
        WsfTimerStartMs(&ButtonTimer, 10);

        //
        // Every time we get a button timer tick, check all of our buttons.
        //
        am_devices_button_array_tick(am_bsp_psButtons, AM_BSP_NUM_BUTTONS);

        //
        // If we got a a press, do something with it.
        //
        if ( am_devices_button_released(am_bsp_psButtons[0]) )
        {
            am_util_debug_printf("Got Button 0 Press : ble_on = true\n");
            ble_on = true;
            AppUiBtnTest(APP_UI_BTN_1_SHORT);
        }

        if ( am_devices_button_released(am_bsp_psButtons[1]) )
        {
            am_util_debug_printf("Got Button 1 Press : ble_on = false\n");
            ble_on = false;
            AppUiBtnTest(APP_UI_BTN_1_SHORT);
        }

        if ( am_devices_button_released(am_bsp_psButtons[2]) )
        {
            am_util_debug_printf("Got Button 2 Press\n");
        }

    }

    if (pMsg->event == POWERCYCLE_TIMER_EVENT)
    {
        // restart timer
        if ( ble_on == true )
        {
            dmConnId_t  connId;

            WsfTimerStartSec(&PowerCycleTimer, 1);

            if ((connId = AppConnIsOpen()) != DM_CONN_ID_NONE)
            {
                AppConnClose(connId);
                return;
            }

            if ( AppSlaveIsAdvertising() == true )
            {
                AppAdvStop();
                return;
            }
            am_util_debug_printf("Power off Apollo3 BLE controller\n");
            HciDrvRadioShutdown();
            ble_on = false;
        }
        else
        {
            am_util_debug_printf("Power on Apollo3 BLE controller\n");
            HciDrvRadioBoot(1);
            DmDevReset();
            ble_on = true;
            WsfTimerStartSec(&PowerCycleTimer, 10);
        }
    }
}

//*****************************************************************************
//
// Sets up a button interface.
//
//*****************************************************************************

typedef enum {
    BLE_IDLE = 0,
    BLE_SWITCHING = 1,
    BLE_POWERSAVE_ING = 2,
    BLE_POWERSAVE_DONE = 3,
    BLE_RESUME_ING = 4,
    BLE_RESUME_DONE = 5,
    BLE_POWEROFF = 6,
    BLE_OTHER = 8
} ble_state_t;
ble_state_t set_ble_state = BLE_IDLE;



/*****************************************************************************/
/*!
 *  \fn     button0_handler
 *
 *  \brief  Handler for the button0 press action.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*****************************************************************************/

void button0_handler(void)
{
}

/*****************************************************************************/
/*!
 *  \fn     button1_handler
 *
 *  \brief  Handler for the button1 press action.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*****************************************************************************/

void button1_handler(void)
{

}

/*****************************************************************************/
/*!
 *  \fn     button2_handler
 *
 *  \brief  Handler for the button1 press action.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*****************************************************************************/

void button2_handler(void)
{
    if (set_ble_state == BLE_IDLE)
    {
        am_util_debug_printf("[button2]set BLE_POWERSAVE\r\n");
        set_ble_state = BLE_POWERSAVE_ING;
    }
    else if (set_ble_state == BLE_POWERSAVE_DONE)
    {
        am_util_debug_printf("[button2]set BLE_RESUME\r\n");
        set_ble_state = BLE_RESUME_ING;
    }
}


void am_gpio_isr(void)
{
    //
    // Read and clear the GPIO interrupt status.
    //
    #if defined(AM_PART_APOLLO3P)
    AM_HAL_GPIO_MASKCREATE(GpioIntStatusMask);

    am_hal_gpio_interrupt_status_get(false, pGpioIntStatusMask);
    am_hal_gpio_interrupt_clear(pGpioIntStatusMask);
    am_hal_gpio_interrupt_service(pGpioIntStatusMask);
    #elif defined(AM_PART_APOLLO3)
    uint64_t ui64Status;

    am_hal_gpio_interrupt_status_get(false, &ui64Status);
    am_hal_gpio_interrupt_clear(ui64Status);
    am_hal_gpio_interrupt_service(ui64Status);
    #else
    #error Unknown device.
    #endif
}

void button_init(void)
{
    am_hal_gpio_interrupt_register(AM_BSP_GPIO_BUTTON0, button0_handler);
    am_hal_gpio_interrupt_register(AM_BSP_GPIO_BUTTON1, button1_handler);
    am_hal_gpio_interrupt_register(AM_BSP_GPIO_BUTTON2, button2_handler);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_BUTTON0, g_AM_BSP_GPIO_BUTTON0);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_BUTTON1, g_AM_BSP_GPIO_BUTTON1);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_BUTTON2, g_AM_BSP_GPIO_BUTTON2);

    //
    // Clear the GPIO Interrupt (write to clear).
    //
    AM_HAL_GPIO_MASKCREATE(GpioIntMask0);
    AM_HAL_GPIO_MASKCREATE(GpioIntMask1);
    AM_HAL_GPIO_MASKCREATE(GpioIntMask2);
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask0, AM_BSP_GPIO_BUTTON0));
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask1, AM_BSP_GPIO_BUTTON1));
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask2, AM_BSP_GPIO_BUTTON2));

    //
    // Enable the GPIO/button interrupt.
    //
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask0, AM_BSP_GPIO_BUTTON0));
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask1, AM_BSP_GPIO_BUTTON1));
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask2, AM_BSP_GPIO_BUTTON2));

    // Enable GPIO interrupts to the NVIC.
    NVIC_SetPriority(GPIO_IRQn, 5);
    NVIC_EnableIRQ(GPIO_IRQn);


    // Enable interrupts to the core.
    am_hal_interrupt_master_enable();
}
void
setup_buttons(void)
{
    button_init();
    //
    // Enable the buttons for user interaction.
    //
    //am_devices_button_array_init(am_bsp_psButtons, AM_BSP_NUM_BUTTONS);

    //
    // Start a timer.
    //
    //ButtonTimer.handlerId = ButtonHandlerId;
    //ButtonTimer.msg.event = BUTTON_TIMER_EVENT;

    //WsfTimerStartSec(&ButtonTimer, 2);


    //
    // Start a timer.
    //
    //PowerCycleTimer.handlerId = ButtonHandlerId;
    //PowerCycleTimer.msg.event = POWERCYCLE_TIMER_EVENT;

    //WsfTimerStartSec(&PowerCycleTimer, 4);

}


//*****************************************************************************
//
// Initialization for the ExactLE stack.
//
//*****************************************************************************
void
exactle_stack_init(void)
{
    wsfHandlerId_t handlerId;
    uint16_t       wsfBufMemLen;
    //
    // Set up timers for the WSF scheduler.
    //
    WsfOsInit();
    WsfTimerInit();

    //
    // Initialize a buffer pool for WSF dynamic memory needs.
    //
    wsfBufMemLen = WsfBufInit(sizeof(g_pui32BufMem), (uint8_t *)g_pui32BufMem, WSF_BUF_POOLS,
               g_psPoolDescriptors);

    if (wsfBufMemLen > sizeof(g_pui32BufMem))
    {
        am_util_debug_printf("Memory pool is too small by %d\r\n",
                             wsfBufMemLen - sizeof(g_pui32BufMem));
    }

    //
    // Initialize the WSF security service.
    //
    SecInit();
    SecAesInit();
    SecCmacInit();
    SecEccInit();

    //
    // Set up callback functions for the various layers of the ExactLE stack.
    //
    handlerId = WsfOsSetNextHandler(HciHandler);
    HciHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(DmHandler);
    DmDevVsInit(0);
    DmAdvInit();
    DmConnInit();
    DmConnSlaveInit();
    DmSecInit();
    DmSecLescInit();
    DmPrivInit();
    DmHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(L2cSlaveHandler);
    L2cSlaveHandlerInit(handlerId);
    L2cInit();
    L2cSlaveInit();

    handlerId = WsfOsSetNextHandler(AttHandler);
    AttHandlerInit(handlerId);
    AttsInit();
    AttsIndInit();
    AttcInit();

    handlerId = WsfOsSetNextHandler(SmpHandler);
    SmpHandlerInit(handlerId);
    SmprInit();
    SmprScInit();
    HciSetMaxRxAclLen(251);

    handlerId = WsfOsSetNextHandler(AppHandler);
    AppHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(TagHandler);
    TagHandlerInit(handlerId);

    ButtonHandlerId = WsfOsSetNextHandler(button_handler);

    handlerId = WsfOsSetNextHandler(HciDrvHandler);
    HciDrvHandlerInit(handlerId);
}

//*****************************************************************************
//
// UART interrupt handler.
//
//*****************************************************************************
void
am_uart_isr(void)
{
    uint32_t ui32Status;

    //
    // Read and save the interrupt status, but clear out the status register.
    //
    ui32Status = UARTn(0)->MIS;
    UARTn(0)->IEC = ui32Status;

}

//*****************************************************************************
//
// Interrupt handler for BLE
//
//*****************************************************************************
void
am_ble_isr(void)
{

    HciDrvIntService();

}

//*****************************************************************************
//
// Perform initial setup for the radio task.
//
//*****************************************************************************
void
RadioTaskSetup(void)
{
    am_util_debug_printf("RadioTask: setup\r\n");


    NVIC_SetPriority(BLE_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);

}

//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************
void
RadioTask(void *pvParameters)
{
#if WSF_TRACE_ENABLED == TRUE
    //
    // Enable ITM
    //
    am_util_debug_printf("Starting wicentric trace:\n\n");
#endif

    //
    // Boot the radio.
    //
    HciDrvRadioBoot(1);

    //
    // Initialize the main ExactLE stack.
    //
    exactle_stack_init();
    
    // uncomment the following to set custom Bluetooth address here
    // {
    //     uint8_t bd_addr[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
    //     HciVscSetCustom_BDAddr(&bd_addr);
    // }


    //
    // Prep the buttons for use
    //
    setup_buttons();

    //
    // Start the "Tag" profile.
    //
    TagStart();

    ble_on = true;

    while (1)
    {
        //
        // Calculate the elapsed time from our free-running timer, and update
        // the software timers in the WSF scheduler.
        //
        wsfOsDispatcher();

    }
}

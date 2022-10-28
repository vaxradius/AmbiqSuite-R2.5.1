//*****************************************************************************
//
//! @file ui_task.c
//!
//! @brief Task to handle optical sensor operation.
//!
//*****************************************************************************
#include "ui_task.h"

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
extern ble_state_t set_ble_state;

//*****************************************************************************
//
// ui task handle.
//
//*****************************************************************************
TaskHandle_t UI_task_handle;


//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************


void configure_BLE_state(ble_state_t _state)
{
    uint8_t connId = 0;
    switch(_state)
    {
        set_ble_state = BLE_SWITCHING;

        case BLE_IDLE:
            break;
        case BLE_POWERSAVE_ING:
        {
            am_util_debug_printf("== Power save BLE S==\n");
            dmConnId_t  connId;

            if ((connId = AppConnIsOpen()) != DM_CONN_ID_NONE)
            {
                AppConnClose(connId);
            }

            if ( AppSlaveIsAdvertising() == true )
            {
                AppAdvStop();
            }
            HciDrvRadioShutdown();

            set_ble_state = BLE_POWERSAVE_DONE;
            am_util_debug_printf("== Power save BLE E==\n");
        }
            break;
        case BLE_RESUME_ING:
        {
            am_util_debug_printf("== Power resume BLE S==\n");

            HciDrvRadioBoot(1);
            DmDevReset();

            set_ble_state = BLE_RESUME_DONE;
            am_util_debug_printf("== Power resume BLE E==\n");

        }
            break;
        case BLE_POWEROFF:
            break;
        default:
            break;
    }
}


void UIInit(void){
}

void UITask(void *pvParameters)
{
    const TickType_t xUIDelay8ms = pdMS_TO_TICKS(8UL); //delay 8ms wait OFN 16ms;
    static float system_time;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1){
        configure_BLE_state(set_ble_state);
        vTaskDelayUntil(&xLastWakeTime, xUIDelay8ms);
    }
}


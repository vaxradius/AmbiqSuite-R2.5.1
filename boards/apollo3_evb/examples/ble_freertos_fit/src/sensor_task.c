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
#include "ble_freertos_fit.h"

//*****************************************************************************
//
// Sensor task handle.
//
//*****************************************************************************
TaskHandle_t sensor_task_handle;

//*****************************************************************************
//
// Timer handling to execute XIP codes
//
//*****************************************************************************
static am_hal_ctimer_config_t g_sTimer =
{
    // Don't link timers.
    0,

    // Set up TimerA.
    (AM_HAL_CTIMER_FN_REPEAT |
     AM_HAL_CTIMER_INT_ENABLE    |
     AM_HAL_CTIMER_HFRC_3MHZ),

    // No configuration for TimerB.
    0,
};

static uint8_t s_ui8Data[40];
static uint8_t s_ui8Cnt = 0;

static void
Ctimer_handler(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR( sensor_task_handle, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static void
init_Ctimer(void)
{
	uint32_t ui32Period;

	//
	// Set up timer A0.
	//
	am_hal_ctimer_clear(0, AM_HAL_CTIMER_TIMERA);
	am_hal_ctimer_config(0, &g_sTimer);

	//HFRC_3MHZ 1500 is 0.5ms 
	ui32Period = (1500 * 15); //7.5ms
	am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, ui32Period,
	                         (ui32Period>>1));

	//
	// Clear the timer Interrupt
	//
	am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);

	//
	// Enable the timer Interrupt.
	//
	am_hal_ctimer_int_register(AM_HAL_CTIMER_INT_TIMERA0, Ctimer_handler);
	am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);

	//
	// Enable the timer interrupt in the NVIC.
	//
	NVIC_EnableIRQ(CTIMER_IRQn);
	NVIC_SetPriority(CTIMER_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);
}

//*****************************************************************************
//
// Perform initial setup for the sensor task.
//
//*****************************************************************************
void
SensorTaskSetup(void)
{
	am_util_debug_printf("SensorTask: setup\r\n");

	am_hal_gpio_state_write(9, AM_HAL_GPIO_OUTPUT_SET); 
	am_hal_gpio_pinconfig(9, g_AM_HAL_GPIO_OUTPUT);
	
	init_Ctimer();


}

void Start_SensorTimer(void)
{
	//
	// Stop timer A0. Just in case host died without sending STOP last time
	//
	am_hal_ctimer_stop(0, AM_HAL_CTIMER_TIMERA);
	//
	// Start timer A0
	//
	am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);
}

void Stop_SensorTimer(void)
{
	am_hal_ctimer_stop(0, AM_HAL_CTIMER_TIMERA);
}

//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************
void
SensorTask(void *pvParameters)
{
	for(;;)
	{
		ulTaskNotifyTake(
                                 pdTRUE,          /* Clear the notification value before exiting. */
                                 portMAX_DELAY ); /* Block indefinitely. */

		am_hal_gpio_state_write(9, AM_HAL_GPIO_OUTPUT_TOGGLE);

		s_ui8Data[0] = 0xA5;
		s_ui8Data[1] = s_ui8Cnt++;
		s_ui8Data[39] = s_ui8Cnt;
		fitSendNotification(40, s_ui8Data);
	}
}

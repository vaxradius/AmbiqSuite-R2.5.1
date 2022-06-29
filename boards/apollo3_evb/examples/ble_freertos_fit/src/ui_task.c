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
// ui task handle.
//
//*****************************************************************************
TaskHandle_t UI_task_handle;

//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************

void UIInit(void){
	//am_hal_gpio_pinconfig(45, g_AM_HAL_GPIO_OUTPUT);
}

void UITask(void *pvParameters)
{
    const TickType_t xUIDelay8ms = pdMS_TO_TICKS(8UL); //delay 8ms wait OFN 16ms;

    static float system_time;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1){
        //taskENTER_CRITICAL();
        //am_hal_gpio_state_write(45, AM_HAL_GPIO_OUTPUT_SET);
        vTaskDelayUntil(&xLastWakeTime, xUIDelay8ms);
        //am_hal_gpio_state_write(45, AM_HAL_GPIO_OUTPUT_CLEAR);
        //taskEXIT_CRITICAL();
    }
}


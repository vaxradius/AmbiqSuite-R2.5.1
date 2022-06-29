#ifndef UI_TASK_H
#define UI_TASK_H

//*****************************************************************************
//
// Required built-ins.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

//*****************************************************************************
//
// Standard AmbiqSuite includes.
//
//*****************************************************************************
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// FreeRTOS include files.
//
//*****************************************************************************
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "portable.h"
#include "semphr.h"
#include "event_groups.h"
//#include "rtos.h"

//*****************************************************************************
//
// Variable define
//
//*****************************************************************************

//*****************************************************************************
//
// Task include files.
//
//*****************************************************************************


//*****************************************************************************
//
// Radio task handle.
//
//*****************************************************************************
extern TaskHandle_t UI_task_handle;
void UIInit(void);
void UITask(void *pvParameters);

#endif // UI_TASK_H

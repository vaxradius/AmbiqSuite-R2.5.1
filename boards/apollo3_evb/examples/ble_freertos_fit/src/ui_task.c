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
TaskHandle_t UIa_task_handle;
TaskHandle_t UIb_task_handle;


//*****************************************************************************
//
//  Purpose:
//
//    prime_number() returns the number of primes between 1 and N.
//
//  Discussion:
//
//    A naive algorithm is used.
//
//    Mathematica can return the number of primes less than or equal to N
//    by the command PrimePi[N].
//
//                N  PRIME_NUMBER
//
//                1           0
//               10           4
//              100          25
//            1,000         168
//           10,000       1,229
//          100,000       9,592
//        1,000,000      78,498
//       10,000,000     664,579
//      100,000,000   5,761,455
//    1,000,000,000  50,847,534
//
//  Licensing:
//
//    This code is distributed under the GNU LGPL license.
//
//  Modified:
//
//    23 April 2009
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int N, the maximum number to check.
//
//    Output, int PRIME_NUMBER, the number of prime numbers up to N.
//
//*****************************************************************************
uint32_t
prime_number(int32_t i32n)
{
    uint32_t ui32Total, ui32Prime;
    int32_t ix, jx;

    ui32Total = 0;

    for ( ix = 2; ix <= i32n; ix++ )
    {
        ui32Prime = 1;
        for ( jx = 2; jx < ix; jx++ )
        {
            if ( (ix % jx) == 0 )
            {
                ui32Prime = 0;
                break;
            }
        }
        ui32Total += ui32Prime;
    }

    return ui32Total;
}

//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************
uint32_t UIaTask_uicount = 0;
void UIaTask(void *pvParameters)
{
	const TickType_t xUIDelayms = pdMS_TO_TICKS(8UL); //delay 8ms 
	uint32_t errcount = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		vTaskDelayUntil(&xLastWakeTime, xUIDelayms);
		if(prime_number(100) != 25)
			errcount++;
		if(++UIaTask_uicount % 500 == 0)
			am_util_debug_printf("UITask_A(%d), err=%d\r\n", UIaTask_uicount,errcount);
	}
}

uint32_t UIbTask_uicount = 0;
void UIbTask(void *pvParameters)
{
	const TickType_t xUIDelayms = pdMS_TO_TICKS(5UL); //delay 5ms 
	uint32_t errcount = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		vTaskDelayUntil(&xLastWakeTime, xUIDelayms);
		if(prime_number(100) != 25)
			errcount++;
		if(++UIbTask_uicount % 500 == 0)
			am_util_debug_printf("UITask_B(%d), err=%d\r\n", UIbTask_uicount,errcount);
	}
}


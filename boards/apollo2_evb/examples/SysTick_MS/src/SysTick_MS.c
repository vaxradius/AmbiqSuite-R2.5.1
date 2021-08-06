#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


#include "am_mcu_apollo.h"
#include "am_util_delay.h"

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
volatile uint64_t g_ui64SysTickWrappedTime = 0;

#define AM_SYSTICK_MAX_LOAD_VALUE 0x00FFFFFF
#define AM_CORECLK_HZ     AM_HAL_CLKGEN_FREQ_MAX_HZ
#define AM_CORECLK_KHZ    (AM_CORECLK_HZ/1000)
#define AM_GET_MS_TICK    (((uint64_t)(AM_SYSTICK_MAX_LOAD_VALUE - am_hal_systick_count()) + g_ui64SysTickWrappedTime)/AM_CORECLK_KHZ)

//*****************************************************************************
//
// Systick ISR.
//
//*****************************************************************************
void
am_systick_isr(void)
{
    /* Add enough cycles to account for one full cycle of systick */
    g_ui64SysTickWrappedTime += 0x01000000;
}

volatile uint32_t g_ui32SysTickLast = AM_SYSTICK_MAX_LOAD_VALUE;
volatile uint64_t g_ui64SysTickWrappedTimeLast = 0;

uint32_t am_timebase_get_tick(void)
{
    uint32_t critical;
    uint32_t tick;
    uint32_t systick;
    uint32_t CoreClk;

    critical = am_hal_interrupt_master_disable();/*Enter Critical Section*/

    CoreClk = AM_CORECLK_KHZ;

    systick = am_hal_systick_count();
    if (g_ui64SysTickWrappedTime == g_ui64SysTickWrappedTimeLast && systick > g_ui32SysTickLast)
        tick =(((AM_SYSTICK_MAX_LOAD_VALUE - systick) + (AM_SYSTICK_MAX_LOAD_VALUE - g_ui32SysTickLast) + g_ui64SysTickWrappedTime)/CoreClk);
    else
        tick =(((AM_SYSTICK_MAX_LOAD_VALUE - systick) + g_ui64SysTickWrappedTime)/CoreClk);

    g_ui32SysTickLast = systick;
    g_ui64SysTickWrappedTimeLast = g_ui64SysTickWrappedTime;

    am_hal_interrupt_master_set(critical);/*Exit Critical Section*/
    return tick;
}

void am_timebase_init(void)
{
    am_hal_systick_load(AM_SYSTICK_MAX_LOAD_VALUE);
    am_hal_systick_int_enable();
    am_hal_systick_start();
}

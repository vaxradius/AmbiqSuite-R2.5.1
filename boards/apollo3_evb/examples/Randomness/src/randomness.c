
//*****************************************************************************
//
// Copyright (c) 2019, Ambiq Micro
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
// This is part of revision v2.2.0-7-g63f7c2ba1 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"


/*
o	A) 100ms timer on LFRC. Measure # of HFRC cycles elapsed (~4,800,000). Repeat 100 times.
o	B) 1s timer on LFRC. Measure # of HFRC cycle elapsed (~48,000,000). Repeat 100 times.
o	C) 10s timer on LFRC. Measure # of HFRC cycles (~480,000,000). Repeat 100 times (17 minutes!).
*/

#define CTIMER_CLOCK_SOURCE AM_HAL_CTIMER_LFRC_512HZ
#define MEASURE_PERIOD 512 //~100ms:51, 1s:512, 10s:5120  
#define REPEAT_TIMES 100


bool g_timer_flag = false;

extern void am_timebase_init(void);
extern uint32_t am_timebase_get_tick(unsigned long long* u64tick);


//*****************************************************************************
//
// Timer configurations.
//
//*****************************************************************************
am_hal_ctimer_config_t g_sTimer0 =
{
    // Don't link timers.
    0,

    // Set up TimerA0.
    (AM_HAL_CTIMER_FN_REPEAT |
     AM_HAL_CTIMER_INT_ENABLE |
     CTIMER_CLOCK_SOURCE),

    // No configuration required for TimerB0.
    0,
};


//*****************************************************************************
//
// Init function for Timer A0.
//
//*****************************************************************************
void
timerA0_init(void)
{
    //
    // Enable the LFRC.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_LFRC_START, 0);

    //
    // Set up timer A0.
    //
    am_hal_ctimer_clear(0, AM_HAL_CTIMER_TIMERA);
    am_hal_ctimer_config(0, &g_sTimer0);

    //
    // Set the timing for timerA0.
    //
    am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, (MEASURE_PERIOD-1), 0);

    //
    // Clear the timer Interrupt
    //
    am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);
}

//*****************************************************************************
//
// Timer Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_ctimer_isr(void)
{
    //
    // Clear TimerA0 Interrupt (write to clear).
    //
    am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);
	g_timer_flag = true;

}

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
	unsigned long long u64tick_now = 0;
	unsigned long long u64tick_last = 0;
	int count = 0;
	//
    // Set the clock frequency.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();


    //
    // Initialize the printf interface for ITM output
    //
    am_bsp_itm_printf_enable();

    //
    // TimerA0 init.
    //
    timerA0_init();

    am_timebase_init();

    //
    // Enable the timer Interrupt.
    //
    am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);

    //
    // Enable the timer interrupt in the NVIC.
    //
    NVIC_EnableIRQ(CTIMER_IRQn);
    am_hal_interrupt_master_enable();

    //
    // Enable the timer.
    //
    am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);

	am_util_stdio_printf("Randomness\n\n");


    //
    // Loop forever, printing when awakened by the timer.
    //
    while (1)
    {
		//
		// Go to Deep Sleep and wait for a wake up.
		//
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
		if(g_timer_flag && count++ < REPEAT_TIMES)
		{
			g_timer_flag = false;
			am_util_stdio_printf("%d.\t(%dms)\t",count,am_timebase_get_tick(&u64tick_now));
			am_util_stdio_printf("%lld\n",u64tick_now-u64tick_last);
			u64tick_last = u64tick_now;
			if(count == REPEAT_TIMES)
				am_util_stdio_printf("done\n\n");
		}
    }

}

//*****************************************************************************
//
//! @file clkout.c
//!
//! @brief Enables a clock source to clkout and then tracks it on an LED array.
//!
//! This example enables the LFRC to a clkout pin then uses GPIO polling to
//! track its rising edge and toggle an LED at 1 hertz.
//
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

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************


//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{

    //
    // Set the clock frequency.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_MAX);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_enable(&am_hal_cachectrl_defaults);

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_util_stdio_printf_init((am_util_stdio_print_char_t) am_bsp_itm_string_print);

    //
    // Initialize the SWO GPIO pin
    //
    am_bsp_pin_enable(ITM_SWO);

    //
    // Enable the ITM.
    //
    am_hal_itm_enable();

    //
    // Enable debug printf messages using ITM on SWO pin
    //
    am_bsp_debug_printf_enable();

    //
    // Clear the terminal and print the banner.
    //
	am_util_stdio_terminal_clear();
	am_util_stdio_printf("CLKOUT to GPIO4 Example\n\n");
	
	am_util_stdio_printf("1. Write 0x47 to the CLKKEY register to enable access to CLK_GEN registers\n");
	AM_REG(CLKGEN, CLKKEY) = 0x47;
	
	am_util_stdio_printf("2. Set the CALXT register field to 0 to insure calibration is not occurring.\n");
	AM_REG(CLKGEN, CALXT) = 0;


	am_util_stdio_printf("3. Select the XT oscillator by setting the REG_CLKGEN_OCTRL_OSEL bit to 0.\n");
	//
	// Enable the XT.
	//
	am_hal_clkgen_osc_start(AM_HAL_CLKGEN_OSC_XT);
	
	am_util_stdio_printf("4. Select the XT or a division of it on a CLKOUT pad.\n");
	//
	// Enable the clockout to the desired pin.
	//
	am_hal_gpio_pin_config(4, AM_HAL_PIN_4_CLKOUT);

	
	//
	// Initialize clkgen to output the selected clock.
	//
	am_hal_clkgen_clkout_enable(AM_HAL_CLKGEN_CLKOUT_CKSEL_XT_DIV2);
	
	am_util_stdio_printf("5. Measure the frequency Fmeas at the CLKOUT pad.\n");
	am_util_stdio_printf("6. Compute the adjustment value required in ppm as ((Fnom – Fmeas)*1000000)/Fmeas = PAdj\n");
	am_util_stdio_printf("7. Compute the adjustment value in steps as PAdj/(1000000/219) = PAdj/(0.9535) = Adj\n");
	am_util_stdio_printf("8. Compare Adj value with min/max range of -976 to 975\n");
	
	am_util_stdio_printf("9. If target Adj is within min and max, set CALXT = Adj\n");
	AM_REG(CLKGEN, CALXT) = (-976 & 0x7FF);
	////AM_REG(CLKGEN, CALXT) = (975 & 0x7FF);
	am_util_stdio_printf("10. Otherwise, the XT frequency is too low to be calibrated.\n");

    //
    // We are done printing. Disable debug printf messages on ITM.
    //
    am_bsp_debug_printf_disable();





    while (1)
    {
        //
        // Go to Sleep and stay there.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
    }

}

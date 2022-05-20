//*****************************************************************************
//
//! Pin connections for the I/O Master board to the I/O Slave board.
//! SPI:
//!     HOST (ios_lram_host)                    SLAVE (ios_lram)
//!     --------------------                    ----------------
//!     GPIO[10] GPIO Interrupt (slave to host) GPIO[4]  GPIO interrupt
//!     GPIO[5]  IOM0 SPI SCK                   GPIO[0]  IOS SPI SCK
//!     GPIO[7]  IOM0 SPI MOSI                  GPIO[1]  IOS SPI MOSI
//!     GPIO[6]  IOM0 SPI MISO                  GPIO[2]  IOS SPI MISO
//!     GPIO[11] IOM0 SPI nCE                   GPIO[3]  IOS SPI nCE
//!     GND                                     GND
//!
//! I2C:
//!     HOST (ios_lram_host)                    SLAVE (ios_lram)
//!     --------------------                    ----------------
//!     GPIO[10] GPIO Interrupt (slave to host) GPIO[4]  GPIO interrupt
//!     GPIO[5]  IOM0 I2C SCL                   GPIO[0]  IOS I2C SCL
//!     GPIO[6]  IOM0 I2C SDA                   GPIO[1]  IOS I2C SDA
//!     GND                                     GND
//! @endverbatim
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

#define USE_SPI             0   // 0 = I2C, 1 = SPI
#define I2C_ADDR            0x10

#define HANDSHAKE_IOS_PIN   4

#define TEST_IOS_XCMP_INT   1

#define AM_HAL_IOS_INT_ERR  (AM_HAL_IOS_INT_FOVFL | AM_HAL_IOS_INT_FUNDFL | AM_HAL_IOS_INT_FRDERR)

#define AM_HAL_IOS_XCMP_INT (AM_HAL_IOS_INT_XCMPWR | AM_HAL_IOS_INT_XCMPWF | AM_HAL_IOS_INT_XCMPRR | AM_HAL_IOS_INT_XCMPRF)

#define AM_IOSTEST_CMD_SEND_DATA    0xFC
#define AM_IOSTEST_CMD_START_DATA   0xFD
#define AM_IOSTEST_CMD_ACK_DATA     0xFE
#define AM_IOSTEST_CMD_STOP_DATA    0xFF


static void *g_pIOSHandle;
volatile bool bIomSendComplete = false;
//*****************************************************************************
//
// Message buffers.
//
// data from the IOS interface, which is only 8 bits wide.
//
//*****************************************************************************
#define AM_IOS_LRAM_SIZE_MAX    0x78
uint8_t g_pIosSendBuf[AM_IOS_LRAM_SIZE_MAX];
#define AM_IOS_HEADER_SIZE          sizeof(sHeader)
#define AM_IOS_MAX_DATA_SIZE        (AM_IOS_LRAM_SIZE_MAX - AM_IOS_HEADER_SIZE)

//*****************************************************************************
//
// SPI Slave Configuration
//
//*****************************************************************************
static am_hal_ios_config_t g_sIOSSpiConfig =
{
	// Configure the IOS in SPI mode.
	.ui32InterfaceSelect = AM_HAL_IOS_USE_SPI,

	// Eliminate the "read-only" section, so an external host can use the
	// entire "direct write" section.
	.ui32ROBase = 0x78,

	// Making the "FIFO" section as big as possible.
	.ui32FIFOBase = 0x80,

	// We don't need any RAM space, so extend the FIFO all the way to the end
	// of the LRAM.
	.ui32RAMBase = 0x100,

	// FIFO Threshold - set to half the size
	.ui32FIFOThreshold = 0x20,
};

//*****************************************************************************
//
// I2C Slave Configuration
//
//*****************************************************************************
am_hal_ios_config_t g_sIOSI2cConfig =
{
	// Configure the IOS in I2C mode.
	.ui32InterfaceSelect = AM_HAL_IOS_USE_I2C | AM_HAL_IOS_I2C_ADDRESS(I2C_ADDR << 1),

	// Eliminate the "read-only" section, so an external host can use the
	// entire "direct write" section.
	.ui32ROBase = 0x78,

	// Set the FIFO base to the maximum value, making the "direct write"
	// section as big as possible.
	.ui32FIFOBase = 0x80,

	// We don't need any RAM space, so extend the FIFO all the way to the end
	// of the LRAM.
	.ui32RAMBase = 0x100,
	// FIFO Threshold - set to half the size
	.ui32FIFOThreshold = 0x40,
};

//*****************************************************************************
//
// Configure the SPI slave.
//
//*****************************************************************************
void ios_set_up(bool bSpi)
{
	if (bSpi)
	{
		// Configure SPI interface
		am_bsp_ios_pins_enable(0, AM_HAL_IOS_USE_SPI);
		//
		// Configure the IOS interface and LRAM structure.
		//
		am_hal_ios_initialize(0, &g_pIOSHandle);
		am_hal_ios_power_ctrl(g_pIOSHandle, AM_HAL_SYSCTRL_WAKE, false);
		am_hal_ios_configure(g_pIOSHandle, &g_sIOSSpiConfig);
	}
	else
	{
		// Configure I2C interface
		am_bsp_ios_pins_enable(0, AM_HAL_IOS_USE_I2C);
		//
		// Configure the IOS interface and LRAM structure.
		//
		am_hal_ios_initialize(0, &g_pIOSHandle);
		am_hal_ios_power_ctrl(g_pIOSHandle, AM_HAL_SYSCTRL_WAKE, false);
		am_hal_ios_configure(g_pIOSHandle, &g_sIOSI2cConfig);
	}

	//
	// Clear out any IOS register-access interrupts that may be active, and
	// enable interrupts for the registers we're interested in.
	//
	am_hal_ios_interrupt_clear(g_pIOSHandle, AM_HAL_IOS_INT_ALL);
	am_hal_ios_interrupt_enable(g_pIOSHandle, AM_HAL_IOS_INT_ERR | AM_HAL_IOS_INT_FSIZE);
#ifdef TEST_IOINTCTL
	am_hal_ios_interrupt_enable(g_pIOSHandle, AM_HAL_IOS_INT_IOINTW);
#endif
#ifdef TEST_IOS_XCMP_INT
	am_hal_ios_interrupt_enable(g_pIOSHandle, AM_HAL_IOS_XCMP_INT);
#endif

	//
	// Set the bit in the NVIC to accept access interrupts from the IO Slave.
	//
	NVIC_EnableIRQ(IOSLAVE_IRQn);

}


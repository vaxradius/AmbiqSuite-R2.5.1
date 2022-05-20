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
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#define IOM_MODULE                  0
#define USE_SPI                     0   // 0 = I2C, 1 = SPI
#define I2C_ADDR                    0x10

#define AM_IOSTEST_CMD_START_DATA   0xFD
#define AM_IOSTEST_CMD_ACK_DATA     0xFE
#define AM_IOSTEST_CMD_STOP_DATA    0xFF

#define AM_IOSTEST_WRITE_CMD_BIT    0x80

#define IOSOFFSET_INTEN             0x78
#define IOSOFFSET_INTSTAT           0x79
#define IOSOFFSET_INTCLR            0x7A
#define IOSOFFSET_INTSET            0x7B
#define IOSOFFSET_HANDSHAKE         0

//*****************************************************************************
//
// Global message buffer for the IO master.
//
//*****************************************************************************

void *g_IOMHandle;
volatile bool g_IOMDMAComplete = false;

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
uint32_t        DMATCBBuffer[1024];

static am_hal_iom_config_t g_sIOMSpiConfig =
{
	.eInterfaceMode = AM_HAL_IOM_SPI_MODE,
	//    .ui32ClockFreq = AM_HAL_IOM_12MHZ,
	//    .ui32ClockFreq = AM_HAL_IOM_8MHZ,
	//    .ui32ClockFreq = AM_HAL_IOM_6MHZ,
	.ui32ClockFreq = AM_HAL_IOM_4MHZ,
	//    .ui32ClockFreq = AM_HAL_IOM_3MHZ,
	//    .ui32ClockFreq = AM_HAL_IOM_2MHZ,
	//    .ui32ClockFreq = AM_HAL_IOM_1_5MHZ,
	//    .ui32ClockFreq = AM_HAL_IOM_1MHZ,
	//    .ui32ClockFreq = AM_HAL_IOM_750KHZ,
	//    .ui32ClockFreq = AM_HAL_IOM_500KHZ,
	//    .ui32ClockFreq = AM_HAL_IOM_400KHZ,
	//    .ui32ClockFreq = AM_HAL_IOM_375KHZ,
	//    .ui32ClockFreq = AM_HAL_IOM_250KHZ,
	//    .ui32ClockFreq = AM_HAL_IOM_100KHZ,
	//    .ui32ClockFreq = AM_HAL_IOM_50KHZ,
	//    .ui32ClockFreq = AM_HAL_IOM_10KHZ,
	.eSpiMode = AM_HAL_IOM_SPI_MODE_0,
	.pNBTxnBuf          = &DMATCBBuffer[0],
	.ui32NBTxnBufLength = sizeof(DMATCBBuffer) / 4
};

static am_hal_iom_config_t g_sIOMI2cConfig =
{
	.eInterfaceMode = AM_HAL_IOM_I2C_MODE,
	.ui32ClockFreq  = AM_HAL_IOM_1MHZ,
	.pNBTxnBuf          = &DMATCBBuffer[0],
	.ui32NBTxnBufLength = sizeof(DMATCBBuffer) / 4
};


static void pfnIOM_LRAM_Callback(void *pCallbackCtxt, uint32_t status)
{
	// Set the DMA complete flag.
	g_IOMDMAComplete = true;
}

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
//
//! Take over default ISR for IOM 0. (Queue mode service)
//
void
am_iomaster0_isr(void)
{
	uint32_t ui32Status;

	if (!am_hal_iom_interrupt_status_get(g_IOMHandle, true, &ui32Status))
	{
		if ( ui32Status )
		{
			am_hal_iom_interrupt_clear(g_IOMHandle, ui32Status);
			am_hal_iom_interrupt_service(g_IOMHandle, ui32Status);
		}
	}
}


void iom_slave_read(bool bSpi, uint32_t offset, uint8_t *pBuf, uint32_t size)
{
	am_hal_iom_transfer_t       Transaction;

	Transaction.ui8Priority     = 1;        // High priority for now.
	Transaction.ui32InstrLen    = 1;
	Transaction.ui32Instr       = offset;
	Transaction.eDirection      = AM_HAL_IOM_RX;
	Transaction.ui32NumBytes    = size;
	Transaction.pui32RxBuffer   = (uint32_t *)pBuf;
	Transaction.bContinue       = false;
	Transaction.ui8RepeatCount  = 0;
	Transaction.ui32PauseCondition = 0;
	Transaction.ui32StatusSetClr = 0;

	if ( bSpi )
	{
		Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;
	}
	else
	{
		Transaction.uPeerInfo.ui32I2CDevAddr = I2C_ADDR;
	}
	g_IOMDMAComplete = false;
	am_hal_iom_nonblocking_transfer(g_IOMHandle, &Transaction, pfnIOM_LRAM_Callback, NULL);
	while (!g_IOMDMAComplete);
}

void iom_slave_write(bool bSpi, uint32_t offset, uint8_t *pBuf, uint32_t size)
{
	am_hal_iom_transfer_t       Transaction;

	Transaction.ui8Priority     = 1;        // High priority for now.
	Transaction.ui32InstrLen    = 1;
	Transaction.ui32Instr       = (AM_IOSTEST_WRITE_CMD_BIT | offset);
	Transaction.eDirection      = AM_HAL_IOM_TX;
	Transaction.ui32NumBytes    = size;
	Transaction.pui32TxBuffer   = (uint32_t *)pBuf;
	Transaction.bContinue       = false;
	Transaction.ui8RepeatCount  = 0;
	Transaction.ui32PauseCondition = 0;
	Transaction.ui32StatusSetClr = 0;

	if ( bSpi )
	{
		Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;
	}
	else
	{
		Transaction.uPeerInfo.ui32I2CDevAddr = I2C_ADDR;
	}
	g_IOMDMAComplete = false;
	am_hal_iom_nonblocking_transfer(g_IOMHandle, &Transaction, pfnIOM_LRAM_Callback, NULL);
	while (!g_IOMDMAComplete);
}

void iom_set_up(uint32_t iomModule, bool bSpi)
{
	uint32_t ioIntEnable;

	//
	// Initialize the IOM.
	//
	am_hal_iom_initialize(iomModule, &g_IOMHandle);

	am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false);

	if ( bSpi )
	{
		//
		// Set the required configuration settings for the IOM.
		//
		am_hal_iom_configure(g_IOMHandle, &g_sIOMSpiConfig);

		//
		// Configure the IOM pins.
		//
		am_bsp_iom_pins_enable(iomModule, AM_HAL_IOM_SPI_MODE);
	}
	else
	{
		//
		// Set the required configuration settings for the IOM.
		//
		am_hal_iom_configure(g_IOMHandle, &g_sIOMI2cConfig);

		//
		// Configure the IOM pins.
		//
		am_bsp_iom_pins_enable(iomModule, AM_HAL_IOM_I2C_MODE);
	}

	// Enable interrupts for NB send to work
	am_hal_iom_interrupt_enable(g_IOMHandle, 0xFF);
	NVIC_EnableIRQ(IOMSTR0_IRQn);

	//
	// Enable the IOM.
	//
	am_hal_iom_enable(g_IOMHandle);

	// Set up IOCTL interrupts
	// IOS ==> IOM
	ioIntEnable = 0xA5;
	iom_slave_write(bSpi, IOSOFFSET_INTEN, (uint8_t*)&ioIntEnable, 1);
	ioIntEnable = 0x00;
	iom_slave_read(bSpi, IOSOFFSET_INTEN, (uint8_t*)&ioIntEnable, 1);
}


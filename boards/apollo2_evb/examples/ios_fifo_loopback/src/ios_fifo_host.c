//*****************************************************************************
//
//! @file ios_fifo_host.c
//!
//! @brief Example host used for demonstrating the use of the IOS FIFO.
//!
//! This host component runs on one EVB and is used in conjunction with
//! the companion slave example, ios_fifo, which runs on a second EVB.
//!
//! The host example uses the ITM SWO to let the user know progress and
//! status of the demonstration.  The SWO is configured at 1M baud.
//! The ios_fifo example has no print output.
//!
//! This example implements the host part of a protocol for data exchange with
//! an Apollo IO Slave (IOS).  The host sends one byte commands on SPI/I2C by
//! writing to offset 0x80.
//!
//! The command is issued by the host to Start/Stop Data accumulation, and also
//! to acknowledge read-complete of a block of data.
//!
//! On the IOS side, once it is asked to start accumulating data (using START
//! command), two CTimer based events emulate sensors sending data to IOS.
//! When IOS has some data for host, it implements a state machine,
//! synchronizing with the host.
//!
//! The IOS interrupts the host to indicate data availability. The host then
//! reads the available data (as indicated by FIFOCTR) by READing using IOS FIFO
//! (at address 0x7F).  The IOS keeps accumulating any new data coming in the
//! background.
//!
//! Host sends an acknowledgement to IOS once it has finished reading a block
//! of data initiated by IOS (partitally or complete). IOS interrupts the host
//! again if and when it has more data for the host to read, and the cycle
//! repeats - till host indicates that it is no longer interested in receiving
//! data by sending STOP command.
//!
//! In order to run this example, a slave device (e.g. a second EVB) must be set
//! up to run the companion example, ios_fifo.  The two boards can be connected
//! using fly leads between the two boards as follows.
//!
//! @verbatim
//! Pin connections for the I/O Master board to the I/O Slave board.
//!
//!     HOST (ios_fifo_host)                    SLAVE (ios_fifo)
//!     --------------------                    ----------------
//!     GPIO[10] GPIO Interrupt (slave to host) GPIO[4]  GPIO interrupt
//!     GPIO[5]  IOM0 SPI CLK/I2C SCL           GPIO[0]  IOS SPI SCK/I2C SCL
//!     GPIO[6]  IOM0 SPI MISO/I2C SDA          GPIO[1]  IOS SPI MISO/I2C SDA
//!     GPIO[7]  IOM0 SPI MOSI                  GPIO[2]  IOS SPI MOSI
//!     GPIO[11] IOM0 SPI nCE                   GPIO[3]  IOS SPI nCE
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "am_devices.h"

#define     IOM_MODULE          0

// How much data to read from Slave before ending the test
#define     MAX_SIZE            10000

#define     XOR_BYTE            0
#define     EMPTY_BYTE          0xEE

typedef enum
{
    AM_IOSTEST_CMD_START_DATA    = 0,
    AM_IOSTEST_CMD_STOP_DATA     = 1,
    AM_IOSTEST_CMD_ACK_DATA      = 2,
} AM_IOSTEST_CMD_E;

#define WRITE_BIT (1 << 7)
#define IOSOFFSET_WRITE_INTEN       (0x78 | WRITE_BIT)
#define IOSOFFSET_WRITE_INTCLR      (0x7A | WRITE_BIT)
#define IOSOFFSET_WRITE_CMD         (0x00 | WRITE_BIT)
#define IOSOFFSET_READ_FW_VER		0x01
#define IOSOFFSET_READ_INTSTAT      0x79
#define IOSOFFSET_READ_FIFO         0x7F
#define IOSOFFSET_READ_FIFOCTR      0x7C

#define AM_IOSTEST_IOSTOHOST_DATAAVAIL_INTMASK  1

#define HANDSHAKE_PIN            10

//*****************************************************************************
//
// Configure GPIOs for communicating with a SPI fram
// TODO: This should ideally come from BSP to keep the code portable
//
//*****************************************************************************
static const uint32_t apollo2_iomce0[AM_REG_IOMSTR_NUM_MODULES][2] =
{
    {11, AM_HAL_PIN_11_M0nCE0},
    {12, AM_HAL_PIN_12_M1nCE0},
#ifdef AM_PART_APOLLO2
    { 3, AM_HAL_PIN_3_M2nCE0},
    {26, AM_HAL_PIN_26_M3nCE0},
    {29, AM_HAL_PIN_29_M4nCE0},
    {24, AM_HAL_PIN_24_M5nCE0}
#endif
};
//*****************************************************************************
//
// Global message buffer for the IO master.
//
//*****************************************************************************
#define AM_TEST_RCV_BUF_SIZE    1024 // Max Size we can receive is 1023
uint8_t g_pui8RcvBuf[AM_TEST_RCV_BUF_SIZE];
volatile uint32_t g_startIdx = 0;
volatile bool bIosInt = false;

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
static am_hal_iom_config_t g_sIOMSpiConfig =
{
    .ui32InterfaceMode = AM_HAL_IOM_SPIMODE,
//    .ui32ClockFrequency = AM_HAL_IOM_12MHZ,
//    .ui32ClockFrequency = AM_HAL_IOM_8MHZ,
//    .ui32ClockFrequency = AM_HAL_IOM_6MHZ,
    .ui32ClockFrequency = AM_HAL_IOM_4MHZ,
//    .ui32ClockFrequency = AM_HAL_IOM_3MHZ,
//    .ui32ClockFrequency = AM_HAL_IOM_2MHZ,
//    .ui32ClockFrequency = AM_HAL_IOM_1_5MHZ,
//    .ui32ClockFrequency = AM_HAL_IOM_1MHZ,
//    .ui32ClockFrequency = AM_HAL_IOM_750KHZ,
//    .ui32ClockFrequency = AM_HAL_IOM_500KHZ,
//    .ui32ClockFrequency = AM_HAL_IOM_400KHZ,
//    .ui32ClockFrequency = AM_HAL_IOM_375KHZ,
//    .ui32ClockFrequency = AM_HAL_IOM_250KHZ,
//    .ui32ClockFrequency = AM_HAL_IOM_100KHZ,
//    .ui32ClockFrequency = AM_HAL_IOM_50KHZ,
//    .ui32ClockFrequency = AM_HAL_IOM_10KHZ,
    .bSPHA = 0,
    .bSPOL = 0,
    .ui8WriteThreshold = 4,
    .ui8ReadThreshold = 60,
};

#define MAX_SPI_SIZE    1023

//*****************************************************************************
//
// Clear Rx Buffer for comparison
//
//*****************************************************************************
void
clear_rx_buf(void)
{
    uint32_t i;
    for ( i = 0; i < AM_TEST_RCV_BUF_SIZE; i++ )
    {
        g_pui8RcvBuf[i] = EMPTY_BYTE;
    }
}

//*****************************************************************************
//
// Validate Rx Buffer
// Returns 0 for success case
//
//*****************************************************************************
uint32_t
validate_rx_buf(uint32_t rxSize)
{
    uint32_t i;
    for ( i = 0; i < rxSize; i++ )
    {
        if ( g_pui8RcvBuf[i] != (((g_startIdx + i) & 0xFF) ^ XOR_BYTE) )
        {
            am_util_stdio_printf("Failed to compare buffers at index %d \n", i);
            break;
        }
    }
    // Set the reference for next chunk
    g_startIdx += rxSize;
    return (i == rxSize);
}

// ISR callback for the host IOINT
static void hostint_handler(void)
{
    bIosInt = true;
}

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void
am_gpio_isr(void)
{
    uint64_t ui64Status;

    //
    // Read and clear the GPIO interrupt status.
    //
    ui64Status = am_hal_gpio_int_status_get(false);
    am_hal_gpio_int_clear(ui64Status);
    am_hal_gpio_int_service(ui64Status);
}

void
iom_slave_read(uint32_t iom, uint32_t offset, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_spi_read(iom, 0,
                        pBuf, size, AM_HAL_IOM_OFFSET(offset));
}

void
iom_slave_write(uint32_t iom, uint32_t offset, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_spi_write(iom, 0,
                         pBuf, size, AM_HAL_IOM_OFFSET(offset));
}

static void
iom_set_up(uint32_t iomModule)
{
    //
    // Enable power to IOM.
    //
    am_hal_iom_pwrctrl_enable(iomModule);

    //
    // Set the required configuration settings for the IOM.
    //
    am_hal_iom_config(iomModule, &g_sIOMSpiConfig);

    //
    // Set up IOM SPI pins. Attributes are set in am_bsp_gpio.h.
    //
    am_bsp_iom_spi_pins_enable(iomModule);

    //
    // Enable the chip-select and data-ready pin.
    //! @note You can enable pins in the HAL or BSP.
    //
    am_hal_gpio_pin_config(apollo2_iomce0[iomModule][0],
        apollo2_iomce0[iomModule][1]);

    //
    // Turn on the IOM for this operation.
    //
    am_bsp_iom_enable(iomModule);

    // Set up the host IO interrupt
    am_hal_gpio_pin_config(HANDSHAKE_PIN, AM_HAL_GPIO_INPUT);
    am_hal_gpio_int_polarity_bit_set(HANDSHAKE_PIN, AM_HAL_GPIO_RISING);
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(HANDSHAKE_PIN));
    // Register handler for IOS => IOM interrupt
    am_hal_gpio_int_register(HANDSHAKE_PIN, hostint_handler);
    am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(HANDSHAKE_PIN));
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);
}

uint32_t g_ui32LastUpdate = 0;

//*****************************************************************************
//
// Print a progress message.
//
//*****************************************************************************
void
update_progress(uint32_t ui32NumPackets)
{
    //
    // Print a dot every 10000 packets.
    //
    if ( (ui32NumPackets - g_ui32LastUpdate) > 1000 )
    {
        am_util_stdio_printf(".");
        g_ui32LastUpdate = ui32NumPackets;
    }
}

void iom_init(void)
{
	uint32_t iom = IOM_MODULE;
    uint32_t data;
	uint32_t ioIntEnable;
	// Set up IOM & Enable interrupt for IOS
    iom_set_up(iom);

	// Read FW version from the offset1 of direct area.
	iom_slave_read(iom, IOSOFFSET_READ_FW_VER,&data, 1);
	am_util_stdio_printf("FW_VER: 0x%X\n", data);

	// Set up IOCTL interrupts
    // IOS ==> IOM
    ioIntEnable = AM_IOSTEST_IOSTOHOST_DATAAVAIL_INTMASK;
    iom_slave_write(iom, IOSOFFSET_WRITE_INTEN, &ioIntEnable, 1);

    // Send the START
    data = AM_IOSTEST_CMD_START_DATA;
    iom_slave_write(iom, IOSOFFSET_WRITE_CMD, &data, 1);
}

void iom_task(void)
{
	uint32_t iom = IOM_MODULE;
    static bool bReadIosData = false;
    static bool bDone = false;
    uint32_t data;
    uint32_t maxSize = MAX_SPI_SIZE;

    if ( !bDone )
    {
        if ( bIosInt == true )
        {
            bIosInt = false;
            // Read & Clear the IOINT status
            iom_slave_read(iom, IOSOFFSET_READ_INTSTAT, &data, 1);
            // We need to clear the bit by writing to IOS
            if ( data & AM_IOSTEST_IOSTOHOST_DATAAVAIL_INTMASK )
            {
                data = AM_IOSTEST_IOSTOHOST_DATAAVAIL_INTMASK;
                iom_slave_write(iom, IOSOFFSET_WRITE_INTCLR, &data, 1);
                // Set bReadIosData
                bReadIosData = true;
            }
            if ( bReadIosData )
            {
                uint32_t iosSize = 0;

                bReadIosData = false;

                // Read the Data Size
                iom_slave_read(iom, IOSOFFSET_READ_FIFOCTR, &iosSize, 2);
                iosSize = (iosSize > maxSize)? maxSize: iosSize;
#if 0
                // Initialize Rx Buffer for later comparison
                clear_rx_buf();
                // Read the data
                iom_slave_read(iom, IOSOFFSET_READ_FIFO,
                    (uint32_t *)g_pui8RcvBuf, iosSize);
				
                // Validate Content
                if ( !validate_rx_buf(iosSize) )
                {
                    am_util_stdio_printf("\nData Verification failed Accum:%lu rx=%d\n",
                        g_startIdx, iosSize);
                }
                // Send the ACK/STOP
                data = AM_IOSTEST_CMD_ACK_DATA;

                update_progress(g_startIdx);

                if ( g_startIdx >= MAX_SIZE )
                {
                    bDone = true;
					am_util_stdio_printf("\nTest Done - Total Received = =%d\n", g_startIdx);
                    data = AM_IOSTEST_CMD_STOP_DATA;
                }
                iom_slave_write(iom, IOSOFFSET_WRITE_CMD, &data, 1);
#else			
				memset(g_pui8RcvBuf,0,sizeof(g_pui8RcvBuf));
				// Read the data
                iom_slave_read(iom, IOSOFFSET_READ_FIFO,
                    (uint32_t *)g_pui8RcvBuf, iosSize);

				am_util_stdio_printf("iosSize = %d\n",iosSize);
				for(int i=0; i < iosSize; i++)
					am_util_stdio_printf("%X ", *(g_pui8RcvBuf+i));
				
				am_util_stdio_printf("\n%s\n\n",g_pui8RcvBuf);
				
				// Send the ACK
                data = AM_IOSTEST_CMD_ACK_DATA;
				iom_slave_write(iom, IOSOFFSET_WRITE_CMD, &data, 1);
#endif				
            }
        }
    }	
}


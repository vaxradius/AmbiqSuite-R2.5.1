/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sensirion_arch_config.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#define IOM_4_SGP40 4

//
// IOM Queue Memory
//
am_hal_iom_queue_entry_t g_psQueueMemory[32];

//*****************************************************************************
//
// I2C Master Configuration
//
//*****************************************************************************
static am_hal_iom_config_t g_sIOMI2cConfig =
{
    .ui32InterfaceMode = AM_HAL_IOM_I2CMODE,
    .ui32ClockFrequency = AM_HAL_IOM_400KHZ,
    .ui8WriteThreshold = 12,
    .ui8ReadThreshold = 120,
};

/*
 * INSTRUCTIONS
 * ============
 *
 * Implement all functions where they are marked as IMPLEMENT.
 * Follow the function specification in the comments.
 */
 
 //
//! Take over default ISR for IOM 4. (Queue mode service)
//
void
am_iomaster4_isr(void)
{
    uint32_t ui32Status;

    ui32Status = am_hal_iom_int_status_get(IOM_4_SGP40, true);

    am_hal_iom_int_clear(IOM_4_SGP40, ui32Status);

    am_hal_iom_queue_service(IOM_4_SGP40, ui32Status);
}

/**
 * Select the current i2c bus by index.
 * All following i2c operations will be directed at that bus.
 *
 * THE IMPLEMENTATION IS OPTIONAL ON SINGLE-BUS SETUPS (all sensors on the same
 * bus)
 *
 * @param bus_idx   Bus index to select
 * @returns         0 on success, an error code otherwise
 */
int16_t sensirion_i2c_select_bus(uint8_t bus_idx) {
    // IMPLEMENT or leave empty if all sensors are located on one single bus
    return STATUS_FAIL;
}

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
void sensirion_i2c_init(void) {
	//
	// Enable power to IOM.
	//
	am_hal_iom_pwrctrl_enable(IOM_4_SGP40);

	//
	// Set the required configuration settings for the IOM.
	//
	am_hal_iom_config(IOM_4_SGP40, &g_sIOMI2cConfig);

	//
	// Set pins high to prevent bus dips.
	//
	am_hal_gpio_out_bit_set(39);
	am_hal_gpio_out_bit_set(40);

	am_hal_gpio_pin_config(39, AM_HAL_PIN_39_M4SCL | AM_HAL_GPIO_PULL12K);
	am_hal_gpio_pin_config(40, AM_HAL_PIN_40_M4SDA | AM_HAL_GPIO_PULL12K);

	am_hal_iom_int_enable(IOM_4_SGP40, 0xFF);
	am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOMASTER0+IOM_4_SGP40);

	//
	// Turn on the IOM for this operation.
	//
	am_bsp_iom_enable(IOM_4_SGP40);


	//
	// Set up the IOM transaction queue.
	//
	am_hal_iom_queue_init(IOM_4_SGP40, g_psQueueMemory, sizeof(g_psQueueMemory));
}

/**
 * Release all resources initialized by sensirion_i2c_init().
 */
void sensirion_i2c_release(void) {
    // IMPLEMENT or leave empty if no resources need to be freed
}

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_read(uint8_t address, uint8_t* data, uint16_t count) {
	int8_t ret = 0;
	ret = am_hal_iom_i2c_read(IOM_4_SGP40, (uint32_t)address,
                    (uint32_t *)data, (uint32_t) count,
                    AM_HAL_IOM_RAW);
    return ret;
}

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_write(uint8_t address, const uint8_t* data,
                           uint16_t count) {
    int8_t ret = 0;
    ret = am_hal_iom_i2c_write(IOM_4_SGP40, (uint32_t)address,
                             (uint32_t *)data, (uint32_t)count, AM_HAL_IOM_RAW);
    return ret;
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * Despite the unit, a <10 millisecond precision is sufficient.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_sleep_usec(uint32_t useconds) {
    am_util_delay_us(useconds);
}

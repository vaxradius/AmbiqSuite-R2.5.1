#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include "sensirion_common.h"
#include "sensirion_i2c.h"
#include "ICE11101.h"


uint16_t tdk_fill_reg_send_buf(uint8_t* buf, uint16_t reg,
                                     const uint16_t* args, uint8_t num_args) {
    uint8_t i;
    uint16_t idx = 0;

    buf[idx++] = (uint8_t)((reg & 0xFF00) >> 8);
    buf[idx++] = (uint8_t)((reg & 0x00FF) >> 0);

    for (i = 0; i < num_args; ++i) {
        buf[idx++] = (uint8_t)((args[i] & 0xFF00) >> 8);
        buf[idx++] = (uint8_t)((args[i] & 0x00FF) >> 0);
    }
    return idx;
}

int16_t tdk_i2c_read_words(uint8_t address, uint16_t* data_words,
                                 uint16_t num_words) {
    int16_t ret;

    ret = sensirion_i2c_read(address, (uint8_t*) data_words , (num_words * TDK_WORD_SIZE));
	
    if (ret != NO_ERROR)
        return ret;

    return NO_ERROR;
}

int16_t tdk_i2c_write_reg_with_args(uint8_t address, uint16_t reg,
                                          const uint16_t* data_words,
                                          uint16_t num_words) {
    uint8_t buf[TDK_MAX_BUFFER_WORDS];
    uint16_t buf_size;

    buf_size = tdk_fill_reg_send_buf(buf, reg, data_words, num_words);
    return sensirion_i2c_write(address, buf, buf_size);
}

int16_t tdk_i2c_delayed_read_cmd(uint8_t address, uint16_t reg,
                                       uint32_t delay_us, uint16_t* data_words,
                                       uint16_t num_words) {
    int16_t ret;
    uint8_t buf[TDK_REG_SIZE];

    tdk_fill_reg_send_buf(buf, reg, NULL, 0);
    ret = sensirion_i2c_write(address, buf, TDK_REG_SIZE);
	
    if (ret != NO_ERROR)
        return ret;

    if (delay_us)
        sensirion_sleep_usec(delay_us);

    return tdk_i2c_read_words(address, data_words, num_words);
}

int16_t tdk_i2c_read_cmd(uint8_t address, uint16_t reg,
                               uint16_t* data_words, uint16_t num_words) {
    return sensirion_i2c_delayed_read_cmd(address, reg, 0, data_words,
                                          num_words);
}




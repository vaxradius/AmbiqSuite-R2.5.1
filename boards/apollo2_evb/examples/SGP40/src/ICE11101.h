#ifndef ICE11101_H
#define ICE11101_H

#ifdef __cplusplus
extern "C" {
#endif

// ICE11101 I2C Address
#define ICE11101_I2C_ADDR 0x34
// Registers Addresses
#define REG_USR_WHO_AM_I 0x0800
#define REG_USR_FIRMWARE_VER 0x0801
#define REG_USR_ODR 0x0802
#define REG_USR_TEMP_AVG_NB 0x0803
#define REG_USR_CO2_AVG_NB 0x0805
#define REG_USR_INT_CONFIG 0x0806
#define REG_USR_STATUS 0x0807
#define REG_USR_CONTROL 0x0808
#define REG_USR_TEMP_DEGC 0x0809
#define REG_USR_CO2_PPM 0x080B
#define REG_USR_CO2_INI_PPM 0x080D

#define TDK_REG_SIZE 2
#define TDK_MAX_BUFFER_WORDS 32
#define TDK_WORD_SIZE 2

int16_t tdk_i2c_write_reg_with_args(uint8_t address, uint16_t reg,
                                          const uint16_t* data_words,
                                          uint16_t num_words);

int16_t tdk_i2c_delayed_read_cmd(uint8_t address, uint16_t reg,
                                       uint32_t delay_us, uint16_t* data_words,
                                       uint16_t num_words);


#ifdef __cplusplus
}
#endif

#endif /* ICE11101_H */


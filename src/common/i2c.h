#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void i2c_init(void);

// Low-level
uint8_t i2c_read(uint8_t addr, uint8_t* data, uint8_t len);
uint8_t i2c_write(uint8_t addr, uint8_t* data, uint8_t len);

// Register-level
uint8_t i2c_read_register(uint8_t addr, uint8_t reg, uint8_t* value);
uint8_t i2c_read_registers(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t len);
uint8_t i2c_write_register(uint8_t addr, uint8_t reg, uint8_t value);

#endif
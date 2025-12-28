#include "i2c.h"
#include "mpu6050.h"

bool mpu6050_test_connection(void) {
    uint8_t who;
    i2c_read_register(MPU6050_ADDR, WHO_AM_I_REG, &who);
    return (who == WHO_AM_I_VAL);
}
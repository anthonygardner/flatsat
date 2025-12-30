#include "i2c.h"
#include "mpu6050.h"

bool mpu6050_test_connection(void) {
    uint8_t who;
    i2c_read_register(MPU6050_ADDR, WHO_AM_I_REG, &who);
    return (who == WHO_AM_I_VAL);
}

bool mpu6050_init(void) {
    // Wake up MPU6050
    i2c_write_register(MPU6050_ADDR, 0x6B, 0x00);
    return mpu6050_test_connection();
}

bool mpu6050_read_accel(int16_t* accel_x, int16_t* accel_y, int16_t* accel_z) {
    // 3B, 3C, 3D, 3E, 3F, 40 = ACCEL_X/Y/ZOUT_H/L
    uint8_t buffer[6];

    i2c_read_registers(MPU6050_ADDR, 0x3B, buffer, 6);

    *accel_x = buffer[0] * 256 + buffer[1];
    *accel_y = buffer[2] * 256 + buffer[3];
    *accel_z = buffer[4] * 256 + buffer[5];

    return true;
}

bool mpu_6050_read_temp(int16_t* temp) {
    // 41, 42 = TEMP_OUT_H/L
    uint8_t buffer[2];

    i2c_read_registers(MPU6050_ADDR, 0x41, buffer, 2);

    *temp = buffer[0] * 256 + buffer[1];

    return true;
}

bool mpu6050_read_gyro(int16_t* gyro_x, int16_t* gyro_y, int16_t* gyro_z) {
    // 43, 44, 45, 46, 47, 48 = GYRO_X/Y/ZOUT_H/L
    uint8_t buffer[6];

    i2c_read_registers(MPU6050_ADDR, 0x43, buffer, 6);

    *gyro_x = buffer[0] * 256 + buffer[1];
    *gyro_y = buffer[2] * 256 + buffer[3];
    *gyro_z = buffer[4] * 256 + buffer[5];

    return true;
}

bool mpu6050_read_all(mpu6050_raw_t* data) {
    uint8_t buffer[14];

    i2c_read_registers(MPU6050_ADDR, 0x3B, buffer, 14);

    data->accel_x = (buffer[0] << 8) | buffer[1];
    data->accel_y = (buffer[2] << 8) | buffer[3];
    data->accel_z = (buffer[4] << 8) | buffer[5];
    data->temp = (buffer[6] << 8) | buffer[7];
    data->gyro_x = (buffer[8] << 8) | buffer[9];
    data->gyro_y = (buffer[10] << 8) | buffer[11];
    data->gyro_z = (buffer[12] << 8) | buffer[13];

    return true;
}
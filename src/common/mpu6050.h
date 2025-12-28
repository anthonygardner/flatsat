#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#define MPU6050_ADDR 0x68
#define WHO_AM_I_REG 0x75
#define WHO_AM_I_VAL 0x68

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} mpu6050_raw_t;

// TODO(acg): Convert raw bits to sensible units
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
} mpu6050_data_t;

bool mpu6050_init(void);

bool mpu6050_test_connection(void);

bool mpu6050_read_accel(int16_t* x, int16_t* y, int16_t* z);
bool mpu6050_read_gyro(int16_t* x, int16_t* y, int16_t* z);
bool mpu_6050_read_temp(int16_t* temp);
bool mpu6050_read_all(mpu6050_data_t* data);

#endif
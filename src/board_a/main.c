#include "can.h"
#include "clock.h"
#include "eth.h"
#include "gpio.h"
#include "i2c.h"
#include "mpu6050.h"
#include "uart.h"
#include "stm32f767xx.h"

#include <stdbool.h>

int main(void) {
    // Setup LEDs
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (1 << 14) | (1 << 28);

    clock_init();
    SysTick_Init();
    __enable_irq();

    led_init();
    uart_init();
    i2c_init();
    can_init();
    eth_init();

    if (mpu6050_init()) {
        uart_print_str("MPU6050 OK\r\n");
    } else {
        uart_print_str("MPU6050 FAIL\r\n");
    }
    
    // uint8_t counter = 0;
    mpu6050_data_t imu_data;

    while (1) {
        // uint8_t can_data[8] = {counter++, 0, 0, 0, 0, 0, 0, 0};
        // can_transmit(0x100, can_data, 1);
        // led_toggle();

        if (eth_get_link_status()) {
            GPIOB->ODR |= (1 << 7);   // Blue LED on
            GPIOB->ODR &= ~(1 << 14); // Red LED off
        } else {
            GPIOB->ODR &= ~(1 << 7); // Blue LED off
            GPIOB->ODR |= (1 << 14); // Red LED on
        }

        mpu6050_read_all(&imu_data);

        uint32_t ts = clock_get_ms();
        
        uart_print_uint32(ts);
        uart_send_char(',');
        uart_print_float(imu_data.accel_x, 2);
        uart_send_char(',');
        uart_print_float(imu_data.accel_y, 2);
        uart_send_char(',');
        uart_print_float(imu_data.accel_z, 2);
        uart_send_char(',');
        uart_print_float(imu_data.temp, 2);
        uart_send_char(',');
        uart_print_float(imu_data.gyro_x, 2);
        uart_send_char(',');
        uart_print_float(imu_data.gyro_y, 2);
        uart_send_char(',');
        uart_print_float(imu_data.gyro_z, 2);
        uart_print_str("\r\n");
        
        for (volatile int i = 0; i < 2000000; i++);
    }
}
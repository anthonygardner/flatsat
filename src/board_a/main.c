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
    clock_init();
    uart_init();
    i2c_init();
    mpu6050_init();

    if (mpu6050_test_connection()) {
        uart_send_char('Y');

        uint8_t who;
        i2c_read_register(MPU6050_ADDR, 0x75, &who);
        uart_print_hex(who);
    } else {
        uart_send_char('N');
    }

    mpu6050_raw_t data;

    while (1) {
        mpu6050_read_all(&data);
        
        uart_print_str("AX:");
        uart_print_int(data.accel_x);
        uart_print_str(" AY:");
        uart_print_int(data.accel_y);
        uart_print_str(" AZ:");
        uart_print_int(data.accel_z);
        uart_print_str("\r\n");
        uart_print_str("\r\n");

        for (volatile int i = 0; i < 500000; i++);
    }
}

// int main(void) {
//     // Setup LEDs
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
//     GPIOB->MODER |= (1 << 14) | (1 << 28);
    
//     clock_init();
//     led_init();
//     uart_init();
//     can_init();
//     eth_init();
    
//     uint8_t counter = 0;
    
//     while (1) {
//         uint8_t data[8] = {counter++, 0, 0, 0, 0, 0, 0, 0};
        
//         can_transmit(0x100, data, 1);
//         led_toggle();

//         if (eth_get_link_status()) {
//             GPIOB->ODR |= (1 << 7); // Blue LED on
//             GPIOB->ODR &= ~(1 << 14); // Red LED off
//         } else {
//             GPIOB->ODR &= ~(1 << 7); // Blue LED off
//             GPIOB->ODR |= (1 << 14); // Red LED on
//         }

//         for (volatile int i = 0; i < 500000; i++);
//     }
// }
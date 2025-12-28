#include "can.h"
#include "clock.h"
#include "eth.h"
#include "gpio.h"
#include "pwm.h"
#include "uart.h"
#include "stm32f767xx.h"

int main(void) {
    uart_init();
    pwm_init();
    pwm_set_duty(540);

    while (1) {}
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

//     while (1) {
//         uint32_t id;
//         uint8_t data[8];
//         uint8_t len;
        
//         if (can_receive(&id, data, &len)) {
//             led_toggle();
//         }

//         if (eth_get_link_status()) {
//             GPIOB->ODR |= (1 << 7); // Blue LED on
//             GPIOB->ODR &= ~(1 << 14); // Red LED off
//         } else {
//             GPIOB->ODR &= ~(1 << 7); // Blue LED off
//             GPIOB->ODR |= (1 << 14); // Red LED on
//         }
//     }
// }
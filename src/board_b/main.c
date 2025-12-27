#include "can.h"
#include "clock.h"
#include "eth.h"
#include "gpio.h"
#include "uart.h"
#include "stm32f767xx.h"

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (1 << 14);

    GPIOB->ODR |= (1 << 7); // on
    clock_init();
    GPIOB->ODR &= ~(1 << 7); // off

    led_init();
    uart_init();
    GPIOB->ODR |= (1 << 7); // on

    can_init();
    GPIOB->ODR &= ~(1 << 7); // off

    eth_init();
    GPIOB->ODR |= (1 << 7); // on

    while (1) {
        uint32_t id;
        uint8_t data[8];
        uint8_t len;
        
        if (can_receive(&id, data, &len)) {
            led_toggle();
        }

        GPIOB->ODR &= ~(1 << 7); // off
    }
}
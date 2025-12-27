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
    
    uint8_t counter = 0;
    
    while (1) {
        uint8_t data[8] = {counter++, 0, 0, 0, 0, 0, 0, 0};
        
        can_transmit(0x100, data, 1);
        
        led_toggle();

        for (volatile int i = 0; i < 500000; i++);

        GPIOB->ODR &= ~(1 << 7); // off
    }
}
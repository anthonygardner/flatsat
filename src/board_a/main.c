#include "can.h"
#include "clock.h"
#include "eth.h"
#include "gpio.h"
#include "uart.h"
#include "stm32f767xx.h"

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (1 << 7) | (1 << 14);
    
    clock_init();
    led_init();
    uart_init();
    can_init();
    eth_init();
    
    uint8_t counter = 0;
    
    while (1) {
        uint8_t data[8] = {counter++, 0, 0, 0, 0, 0, 0, 0};
        
        can_transmit(0x100, data, 1);
        led_toggle();

        if (eth_get_link_status()) {
            GPIOB->ODR |= (1 << 7); // Blue LED on
        } else {
            GPIOB->ODR |= (1 << 14); // Red LED on
        }

        for (volatile int i = 0; i < 500000; i++);
    }
}
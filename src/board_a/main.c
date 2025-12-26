#include "can.h"
#include "clock.h"
#include "eth.h"
#include "gpio.h"
#include "stm32f767xx.h"

int main(void) {
    // "I'm alive" sanity check
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (1 << 0);
    GPIOB->ODR |= (1 << 0);

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
        for (volatile int i = 0; i < 500000; i++);
    }
}
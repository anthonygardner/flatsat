#include "can.h"
#include "clock.h"
#include "eth.h"
#include "gpio.h"
#include "uart.h"
#include "stm32f767xx.h"

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (1 << 14);
    GPIOB->ODR |= (1 << 7); // Blue LED on

    clock_init();
    led_init();
    uart_init();

    uart_send_char('H');
    uart_send_char('i');
    uart_send_char('\r');
    uart_send_char('\n');

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
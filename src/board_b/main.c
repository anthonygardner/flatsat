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

    while (1) {
        uint32_t id;
        uint8_t data[8];
        uint8_t len;
        
        if (can_receive(&id, data, &len)) {
            led_toggle();
        }
    }
}
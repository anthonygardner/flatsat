#include "uart.h"
#include "stm32f767xx.h"

static void uart_enable_clocks(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
};

static void uart_configure_pins(void) {
    // PD8: TX
    GPIOD->MODER &= ~(0b11 << 16);
    GPIOD->MODER |= (0b10 << 16);
    GPIOD->OSPEEDR |= (0b10 << 16);
    GPIOD->AFR[1] |= (7 << 0);
};

void uart_send_char(char c) {
    // Wait for TXE
    while (!(USART3->ISR & (1 << 7)));

    USART3->TDR = c;
};

void uart_print_hex(uint32_t val) {
    char hex[] = "0123456789ABCDEF";

    for (int i = 28; i >= 0; i -= 4) {
        uart_send_char(hex[(val >> i) & 0xF]);
    }

    uart_send_char('\r');
    uart_send_char('\n');
};

void uart_init(void) {
    uart_enable_clocks();
    uart_configure_pins();

    // Configure baud rate (48 MHz / 115200)
    USART3->BRR = 417;

    // Enable transmitter and USART3
    USART3->CR1 |= (1 << 0) | (1 << 3);
}
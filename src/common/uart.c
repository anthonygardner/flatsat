#include "uart.h"
#include "stm32f767xx.h"

static void uart_enable_clocks(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
};

static void uart_configure_pins(void) {
    // PC6: TX
    GPIOC->MODER &= ~(0b11 << 12);
    GPIOC->MODER |= (0b10 << 12);

    GPIOC->OSPEEDR |= (0b10 << 12);

    GPIOC->AFR[0] &= ~(0xF << 24); // Clear AF bits
    GPIOC->AFR[0] |= (8 << 24); // Set AF8 for USART6
};

void uart_send_char(char c) {
    // Wait for TXE
    while (!(USART6->ISR & (1 << 7)));
    USART6->TDR = c;
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

    // Configure baud rate
    USART6->BRR = 833;

    // Enable transmitter and USART6
    USART6->CR1 |= (1 << 3) | (1 << 13);
}
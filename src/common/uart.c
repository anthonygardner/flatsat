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
    while (!(USART3->ISR & (1 << 7))); // Wait for TXE
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

char uart_get_char(void) {
    while (!(USART3->ISR & (1 << 5))); // Wait until RXNE is set
    return USART3->RDR;
}

void uart_print_int(int16_t val) {
    char buf[7];
    int i = 0;

    if (val < 0) {
        uart_send_char('-');
        val = -val;
    }

    do {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    } while (val > 0);

    while (i > 0) {
        uart_send_char(buf[--i]);
    }
}

void uart_print_str(const char* s) {
    while (*s) {
        uart_send_char(*s++);
    }
}

void uart_init(void) {
    uart_enable_clocks();
    uart_configure_pins();

    // Configure baud rate (48 MHz / 115200)
    USART3->BRR = 417;

    // Enable transmitter, receiver, and USART3
    USART3->CR1 |= (1 << 0) | (1 << 2) | (1 << 3);
}
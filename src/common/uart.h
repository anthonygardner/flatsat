#ifndef UART_H
#define UART_H

#include <stdint.h>

void uart_init(void);

void uart_send_char(char c);

void uart_print_hex(uint32_t val);

char uart_get_char(void);

void uart_print_int(int16_t val);

void uart_print_string(const char* s);

#endif
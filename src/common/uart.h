#ifndef UART_H
#define UART_H

#include <stdint.h>

void uart_init(void);

void uart_send_char(char c);

void uart_print_hex(uint32_t val);

void uart_print_hex8(uint8_t val);

char uart_get_char(void);

void uart_print_int(int16_t val);

void uart_print_uint32(uint32_t val);

void uart_print_float(float val, int dec);

void uart_print_str(const char* s);

void uart_print_char(char c);

#endif
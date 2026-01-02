#ifndef CLOCK_H
#define CLOCK_H

#include <stdint.h>

void clock_init(void);

void SysTick_Init(void);

void SysTick_Handler(void);

uint32_t clock_get_ms(void);

#endif
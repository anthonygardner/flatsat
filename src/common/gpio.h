#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

void gpio_led_init(void);
void gpio_led_toggle(void);

void gpio_motor_init(void);
void gpio_motor_forward(void);
void gpio_motor_reverse(void);
void gpio_motor_stop(void);

void gpio_relay_init(void);
void gpio_relay_toggle_parallel(void);
void gpio_relay_toggle_series(void);

#endif
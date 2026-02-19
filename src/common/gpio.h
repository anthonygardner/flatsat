#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include <stdint.h>

void gpio_led_init(void);
void gpio_led_toggle(void);

void gpio_motor_init(void);
void gpio_motor_forward(void);
void gpio_motor_reverse(void);
void gpio_motor_stop(void);
int16_t gpio_motor_read_encoder(void);
int16_t gpio_motor_get_rpm(void);

void gpio_relay_init(void);
void gpio_relay_toggle_parallel(void);
void gpio_relay_toggle_series(void);

#endif
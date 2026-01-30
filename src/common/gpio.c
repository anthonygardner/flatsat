#include "gpio.h"
#include "stm32f767xx.h"

void gpio_led_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    
    GPIOB->MODER &= ~GPIO_MODER_MODER0;
    GPIOB->MODER |= (0x1 << GPIO_MODER_MODER0_Pos);
}

void gpio_led_toggle(void) {
    GPIOB->ODR ^= GPIO_ODR_OD0;
}

void gpio_motor_init(void) {
    // Enable GPIOF clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;

    // PF12 as output (IN1)
    GPIOF->MODER &= ~(0b11 << 24);
    GPIOF->MODER |= (0b01 << 24);

    // PF13 as output (IN2)
    GPIOF->MODER &= ~(0b11 << 26);
    GPIOF->MODER |= (0b01 << 26);
}

void gpio_motor_forward(void) {
    GPIOF->ODR |= (1 << 12);  // IN1 high
    GPIOF->ODR &= ~(1 << 13); // IN2 low
}

void gpio_motor_reverse(void) {
    GPIOF->ODR &= ~(1 << 12); // IN1 low
    GPIOF->ODR |= (1 << 13);  // IN2 high
}

void gpio_motor_stop(void) {
    GPIOF->ODR &= ~(1 << 12); // IN1 low
    GPIOF->ODR &= ~(1 << 13); // IN2 low
}

void gpio_relay_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    // Set inputs high since relay energizes when pulled to ground
    GPIOE->ODR |= (1 << 8) | (1 << 10) | (1 << 12) | (1 << 14);

    // PE8
    GPIOE->MODER &= ~(0b11 << 16);
    GPIOE->MODER |= (0b01 << 16);

    // PE10
    GPIOE->MODER &= ~(0b11 << 20);
    GPIOE->MODER |= (0b01 << 20);

    // PE12
    GPIOE->MODER &= ~(0b11 << 24);
    GPIOE->MODER |= (0b01 << 24);

    // PE14
    GPIOE->MODER &= ~(0b11 << 28);
    GPIOE->MODER |= (0b01 << 28);
}

void gpio_relay_toggle_series(void) {
    GPIOE->ODR |= 1 << 8; // PE8 high
    GPIOE->ODR |= 1 << 10; // PE10 high - LED off

    for (volatile int i = 0; i < 10000000; i++);

    GPIOE->ODR &= ~(1 << 8); // PE8 low - LED off

    for (volatile int i = 0; i < 10000000; i++);

    GPIOE->ODR &= ~(1 << 10); // PE10 low - LED on

    for (volatile int i = 0; i < 10000000; i++);

    GPIOE->ODR |= 1 << 8; // PE8 high
    GPIOE->ODR |= 1 << 10; // PE10 high - LED off
}

void gpio_relay_toggle_parallel(void) {
    GPIOE->ODR |= (1 << 12); // PE12 high
    GPIOE->ODR |= (1 << 14); // PE14 high - LED off

    for (volatile int i = 0; i < 10000000; i++);

    GPIOE->ODR &= ~(1 << 12); // PE12 low - LED on

    for (volatile int i = 0; i < 10000000; i++);

    GPIOE->ODR |= (1 << 12); // PE12 high - LED off

    for (volatile int i = 0; i < 10000000; i++);

    GPIOE->ODR &= ~(1 << 14); // PE14 low - LED on

    for (volatile int i = 0; i < 10000000; i++);

    GPIOE->ODR |= (1 << 14); // PE14 high - LED off
}
#include "gpio.h"
#include "uart.h"
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
    // Enable GPIOF clock (for PWM out)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;

    // Enable GPIOA clock (for encoder in)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Enable TIM3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // PF12 as output (IN1)
    GPIOF->MODER &= ~(0b11 << 24);
    GPIOF->MODER |= (0b01 << 24);

    // PF13 as output (IN2)
    GPIOF->MODER &= ~(0b11 << 26);
    GPIOF->MODER |= (0b01 << 26);

    // PA6 and PA7 as alternate functions (encoders A and B)
    GPIOA->MODER &= ~(0b11 << 12);
    GPIOA->MODER |= (0b10 << 12);

    GPIOA->MODER &= ~(0b11 << 14);
    GPIOA->MODER |= (0b10 << 14);

    // Tell the chip which AF register to use for PA6 and PA7
    GPIOA->AFR[0] &= ~(0b1111 << 24);
    GPIOA->AFR[0] |= (0b0010 << 24);

    GPIOA->AFR[0] &= ~(0b1111 << 28);
    GPIOA->AFR[0] |= (0b0010 << 28);

    // Configure TIM3 for encoder mode
    TIM3->SMCR |= (0b011 << 0);

    // Tell TIM3 that channels 1 and 2 are direct inputs from the pins
    TIM3->CCMR1 |= (0b01 << 0) | (0b01 << 8);

    // Set counter wrap around value to 65535
    TIM3->ARR = 0xFFFF;

    // Enable the counter
    TIM3->CR1 |= (0b01 << 0);
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

int16_t gpio_motor_read_encoder(void) {
    uint32_t cnt = TIM3->CNT;
    return (int16_t)cnt;
}

int16_t gpio_motor_get_rpm(void) {
    // 12 pulses per rev from spec sheet
    // Need revolutions per minute => (counts / interval) * (1 rev / total counts) * (100 intervals / sec) * (60 sec / 1 min)
    int16_t rpm = gpio_motor_read_encoder();
    return rpm;
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
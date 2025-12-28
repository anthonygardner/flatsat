#include "pwm.h"
#include "stm32f767xx.h"

static void pwm_enable_clocks(void) {
    // Targeting PA0, so enable GPIOA and TIM2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
}

static void pwm_configure_pins(void) {
    // Clear PA0 mode bits
    GPIOA->MODER &= ~(0b11 << 0);

    // Set to alternate function
    GPIOA->MODER |= (0b10 << 0);

    // Clear AF bits for PA0
    GPIOA->AFR[0] &= ~(0xF << 0);

    // Select TIM2_CH1 to AF1
    GPIOA->AFR[0] |= (1 << 0);
}

static void pwm_init_timer(void) {
    // Clock speed of Nucleo F767ZI is 108 MHz
    // Rule of thumb PWM frequency = 10 kHz
    // PWM = Timer clock / ((Prescaler + 1) * (Autoreload + 1))
    // 10,800 = (PS + 1) * (AR + 1)
    // 10,800 = 10 * 1080 = (PS + 1) * (AR + 1) => PS = 9, AR = 1079
    // Note: Higher auto-reload value = finer duty cycle resolution
    
    // Set prescaler value
    TIM2->PSC = 9;

    // Set auto-reload value
    TIM2->ARR = 1079;

    // Select PWM mode using capture / compare mode register
    TIM2->CCMR1 = (0b110 << 4);

    // Enable TIM2 output using capture / compare enable register
    TIM2->CCER = (1 << 0);

    // Start the timer using peripheral control register
    TIM2->CR1 = (1 << 0);
}

void pwm_set_duty(uint16_t duty) {
    // Set duty cycle using capture / compare register
    TIM2->CCR1 = duty;
}

void pwm_init(void) {
    pwm_enable_clocks();
    pwm_configure_pins();
    pwm_init_timer();
}
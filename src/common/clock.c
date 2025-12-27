#include "clock.h"
#include "stm32f767xx.h"

void clock_init(void) {
    // Enable high speed internal (HSI) oscillator clock, wait for ready
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    GPIOB->ODR ^= (1 << 7);

    // Enable power controller
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // Enable high speed external (HSE) oscillator clock, wait for ready
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    GPIOB->ODR ^= (1 << 7);

    // Set flash wait states
    FLASH->ACR |= (0b011 << 0);

    // Configure PLL
    RCC->PLLCFGR = (8 << 0)
                 | (192 << 6)
                 | (0b00 << 16)
                 | (1 << 22);

    // Set APB1 prescaler (APB1 = HCLK / 2 = 48 MHz)
    RCC->CFGR |= (0b100 << 10);

    // Enable PLL, wait for ready
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    GPIOB->ODR ^= (1 << 7);

    // Switch system clock switch to PLL
    RCC->CFGR |= (0b10 << 0);

    // Wait for confirmation from system clock switch status
    while ((RCC->CFGR & (0b11 << 2)) != (0b10 << 2));

    GPIOB->ODR ^= (1 << 7);
}
#include "clock.h"
#include "stm32f767xx.h"

static volatile uint32_t msTicks = 0;

void clock_init(void) {
    // Handle floating point math with FPU
    SCB->CPACR |= ((3UL << 20) | (3UL << 22));

    // Set vector table location to flash
    SCB->VTOR = 0x08000000;

    // Enable high speed internal (HSI) oscillator clock, wait for ready
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // Enable power controller
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // Enable high speed external (HSE) oscillator clock, wait for ready
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

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

    // Set system clock switch to PLL
    RCC->CFGR |= (0b10 << 0);

    // Wait for confirmation from system clock switch status
    while ((RCC->CFGR & (0b11 << 2)) != (0b10 << 2));
}

void SysTick_Init(void) {
    // From ARM Cortex-M4 Technical Reference Manual, Section 4.2
    // SYST_CSR (Control and Status Register): Enable, clock source, interrupt
    // SYST_RVR (Reload Value Register): Value to reload when counter reaches 0
    // SYST_CVR (Current Value Register): Current counter value
    // SYST_CALIB (Calibration Value Register): Calibration info

    // Disable during configuration
    SysTick->CTRL = 0;

    // Set reload value for 1ms tick @ 96MHz
    SysTick->LOAD = 96000 - 1;

    // Clear current value register
    SysTick->VAL = 0;

    // Configure and enable SysTick
    SysTick->CTRL = (1 << 2) |  // CLKSOURCE = 1 (SysTick_CTRL_CLKSOURCE)
                    (1 << 1) |  // TICKINT = 1 (SysTick_CTRL_TICKINT)
                    (1 << 0);   // ENABLE = 1 (SysTick_CTRL_ENABLE)
}

void SysTick_Handler(void) {
    msTicks++;
}

uint32_t clock_get_ms(void) {
    return msTicks;
}
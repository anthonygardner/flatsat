#include "stm32f767xx.h"

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (1 << 14);  // PB7 output
    
    while (1) {
        GPIOB->ODR ^= (1 << 7);
        for (volatile int i = 0; i < 1000000; i++);
    }
}
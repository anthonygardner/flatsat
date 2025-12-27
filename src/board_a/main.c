#include "stm32f767xx.h"

int main(void) {
    // Enable GPIOC clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    
    // Enable USART6 clock
    RCC->APB2ENR |= (1 << 5);
    
    // PC6 as alternate function
    GPIOC->MODER &= ~(0b11 << 12);
    GPIOC->MODER |= (0b10 << 12);
    GPIOC->AFR[0] &= ~(0xF << 24);
    GPIOC->AFR[0] |= (8 << 24);
    
    // Baud rate for 32 MHz HSI
    USART6->BRR = 270;
    
    // Enable TX and USART
    USART6->CR1 = (1 << 0) | (1 << 3);
    
    // Send 'U' forever
    while (1) {
        while (!(USART6->ISR & (1 << 7)));
        USART6->TDR = 'U';
        for (volatile int i = 0; i < 500000; i++);
    }
}
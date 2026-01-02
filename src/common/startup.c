#include <stdint.h>

extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _estack;

extern int main(void);
extern void SysTick_Handler(void);

void Reset_Handler(void);
void Default_Handler(void);

void Reset_Handler(void) {
    uint32_t *src = &_etext;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }

    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }

    main();

    while (1);
}

void Default_Handler(void) {
    while (1);
}

__attribute__((section(".isr_vector")))
void (*vectors[])(void) = {
    (void (*)(void))&_estack,
    Reset_Handler,
    Default_Handler,  // NMI
    Default_Handler,  // HardFault
    Default_Handler,  // MemManage
    Default_Handler,  // BusFault
    Default_Handler,  // UsageFault
    0, 0, 0, 0,
    Default_Handler,  // SVCall
    Default_Handler,  // Debug
    0,
    Default_Handler,  // PendSV
    SysTick_Handler,  // SysTick
};
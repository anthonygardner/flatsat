#include "stm32f767xx.h"
#include "clock.h"
#include "gpio.h"
#include "can.h"

int main(void) {
    clock_init();
    led_init();
    can_init();
    
    while (1) {
        led_toggle();
        for (volatile int i = 0; i < 500000; i++);
    }
}
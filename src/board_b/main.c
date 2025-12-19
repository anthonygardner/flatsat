#include "can.h"
#include "clock.h"
#include "gpio.h"
#include "stm32f767xx.h"

int main(void) {
    clock_init();
    led_init();
    can_init();
    
    while (1) {
        uint32_t id;
        uint8_t data[8];
        uint8_t len;
        
        if (can_receive(&id, data, &len)) {
            led_toggle();
        }
    }
}
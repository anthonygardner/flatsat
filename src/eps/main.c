#include "clock.h"
#include "gpio.h"

int main(void) {
    gpio_relay_init();

    while(1) {
        // IN1 and IN2 are in series and wired to PE8 and PE10
        // gpio_relay_toggle_series();

        // IN3 and IN4 are in parallel and wired to PE12 and PE14
        // gpio_relay_toggle_parallel();
    }

    return 0;
}
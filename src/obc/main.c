#include "can.h"
#include "can_bus.h"
#include "clock.h"
#include "eth.h"
#include "gpio.h"
#include "i2c.h"
#include "obc_config.h"
#include "uart.h"
#include "stm32f767xx.h"
#include <stdbool.h>

static struct {
    uint8_t mode;
    uint8_t error_flags;
} obc_state = {
    .mode = OBC_MODE_IDLE,
    .error_flags = 0
};

static struct {
    uint32_t heartbeat;
    uint32_t eth_check;
} last_run = {0};

static struct {
    uint32_t last_adcs_heartbeat;
    uint32_t last_eps_heartbeat;
} subsystem_health = {0};

static void task_obc_heartbeat(uint32_t now) {
    struct can_bus_obc_heartbeat_t heartbeat = {
        .uptime_ms = now,
        .mode = obc_state.mode,
        .error_flags = obc_state.error_flags
    };

    uint8_t data[CAN_BUS_OBC_HEARTBEAT_LENGTH];
    can_bus_obc_heartbeat_pack(data, &heartbeat, sizeof(data));
    can_transmit(CAN_BUS_OBC_HEARTBEAT_FRAME_ID, data, CAN_BUS_OBC_HEARTBEAT_LENGTH);
}

static void task_eth_status(void) {
    if (eth_get_link_status()) {
        GPIOB->ODR |= (1 << 7);
        GPIOB->ODR &= ~(1 << 14);
    } else {
        GPIOB->ODR &= ~(1 << 7);
        GPIOB->ODR |= (1 << 14);
    }
}

static void task_can_rx(void) {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
    
    while (can_receive(&id, data, &len)) {
        switch (id) {
            case CAN_BUS_ADCS_HEARTBEAT_FRAME_ID: {
                struct can_bus_adcs_heartbeat_t msg;
                can_bus_adcs_heartbeat_unpack(&msg, data, len);
                subsystem_health.last_adcs_heartbeat = clock_get_ms();
                obc_state.error_flags &= ~ERR_ADCS_TIMEOUT;
                break;
            }

            case CAN_BUS_EPS_HEARTBEAT_FRAME_ID: {
                struct can_bus_eps_heartbeat_t msg;
                can_bus_eps_heartbeat_unpack(&msg, data, len);
                subsystem_health.last_eps_heartbeat = clock_get_ms();
                obc_state.error_flags &= ~ERR_EPS_TIMEOUT;
                break;
            }
        }
    }
}

static void obc_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (1 << 14) | (1 << 28);

    clock_init();
    SysTick_Init();
    __enable_irq();

    led_init();
    uart_init();
    i2c_init();
    can_init();
    eth_init();

    uint32_t now = clock_get_ms();
    subsystem_health.last_adcs_heartbeat = now;
    subsystem_health.last_eps_heartbeat = now;
}

int main(void) {
    obc_init();

    while (1) {
        uint32_t now = clock_get_ms();

        // Ethernet status @ 10 Hz
        if (now - last_run.eth_check >= ETH_CHECK_INTERVAL_MS) {
            task_eth_status();
            last_run.eth_check = now;
        }

        // Heartbeats @ 1 Hz
        if (now - last_run.heartbeat >= OBC_HEARTBEAT_INTERVAL_MS) {
            task_obc_heartbeat(now);
            last_run.heartbeat = now;
        }

        // Every loop (non-blocking)
        task_can_rx();

        if (now - subsystem_health.last_adcs_heartbeat > OBC_HEARTBEAT_TIMEOUT_MS) {
            obc_state.error_flags |= ERR_ADCS_TIMEOUT;
        }

        if (now - subsystem_health.last_eps_heartbeat > OBC_HEARTBEAT_TIMEOUT_MS) {
            obc_state.error_flags |= ERR_EPS_TIMEOUT;
        }
    }
}
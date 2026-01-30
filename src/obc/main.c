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
#include <stdlib.h>
#include <string.h>

#define ETH_RX_BUFFER_SIZE 1524
#define UART_BUFFER_SIZE   32

static struct {
    char data[UART_BUFFER_SIZE];
    uint8_t length;
} uart_buffer = {
    .data = {0},
    .length = 0
};

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

static bool handle_motor_cmd(uint8_t duty) {
    if (can_bus_adcs_motor_cmd_duty_cycle_is_in_range(duty)) {
        // Fill struct with values
        struct can_bus_adcs_motor_cmd_t msg = {
            .duty_cycle = duty,
            .enable = 1
        }; 

        // Pack struct into a byte array
        uint8_t data[CAN_BUS_ADCS_MOTOR_CMD_LENGTH];
        can_bus_adcs_motor_cmd_pack(data, &msg, sizeof(data));

        // Transmit byte array over CAN
        return can_transmit(CAN_BUS_ADCS_MOTOR_CMD_FRAME_ID, data, CAN_BUS_ADCS_MOTOR_CMD_LENGTH);
    } else {
        return false;
    }
}

static void task_uart_rx(void) {
    // Check receive not empty register
    if (!(USART3->ISR & (1 << 5))) {
        return;
    }

    // Prevent buffer overflow
    if (uart_buffer.length >= UART_BUFFER_SIZE) {
        memset(uart_buffer.data, 0, sizeof(uart_buffer.data));
        uart_buffer.length = 0;
    }

    // Read (and clear) the register
    char byte = USART3->RDR;

    // Look for end of character sequence
    if (byte != '\n') {
        uart_buffer.data[uart_buffer.length] = byte;
        uart_buffer.length++;
    } else {
        uart_buffer.data[uart_buffer.length] = '\0';

        if (strncmp(uart_buffer.data, "motor", 5) == 0) {
            int duty = atoi(uart_buffer.data + 6);
            handle_motor_cmd(duty);
        }
    }
}

static void task_eth_rx(void) {
    uint8_t buffer[ETH_RX_BUFFER_SIZE];
    uint16_t length;

    if (eth_receive(buffer, &length)) {
        if (buffer[12] == 0x88 && buffer[13] == 0xB5)
        {
            // Check if it's a motor command
            if (strncmp((char *)&buffer[14], "motor ", 6) == 0)
            {
                int duty = atoi((char *)&buffer[20]);
                handle_motor_cmd(duty);
                uart_print_str("Motor cmd sent: ");
                uart_print_int(duty);
                uart_print_str("\r\n");
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

    gpio_led_init();
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
        task_uart_rx();
        task_eth_rx();
        
        if (now - subsystem_health.last_adcs_heartbeat > OBC_HEARTBEAT_TIMEOUT_MS) {
            obc_state.error_flags |= ERR_ADCS_TIMEOUT;
        }

        if (now - subsystem_health.last_eps_heartbeat > OBC_HEARTBEAT_TIMEOUT_MS) {
            obc_state.error_flags |= ERR_EPS_TIMEOUT;
        }
    }

    return 0;
}
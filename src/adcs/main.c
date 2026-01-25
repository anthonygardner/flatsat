#include "adcs_config.h"
#include "can.h"
#include "can_bus.h"
#include "clock.h"
#include "eth.h"
#include "gpio.h"
#include "i2c.h"
#include "mpu6050.h"
#include "pwm.h"
#include "uart.h"
#include "stm32f767xx.h"

static struct {
    uint8_t mode;
    uint8_t error_flags;
} adcs_state = {
    .mode = ADCS_MODE_IDLE,
    .error_flags = 0
};

static struct {
    uint16_t cmd_duty_cycle;
    uint8_t cmd_enable;
    uint16_t actual_rpm;
    float current;
    uint8_t fault;
} motor = {0};

static struct {
    float accel[3];
    float gyro[3];
    float temp;
} imu = {0};

static struct {
    float roll;
    float pitch;
    float yaw;
} attitude = {0};

static struct {
    uint32_t last_obc_heartbeat;
} subsystem_health = {0};

static struct {
    uint32_t heartbeat;
    uint32_t eth_check;
    uint32_t imu_read;
    uint32_t motor_control;
    uint32_t attitude_tx;
    uint32_t motor_status_tx;
} last_run = {0};

static void task_adcs_heartbeat(uint32_t now) {
    struct can_bus_adcs_heartbeat_t hb = {
        .uptime_ms = now,
        .mode = adcs_state.mode,
        .error_flags = adcs_state.error_flags
    };
    
    uint8_t data[CAN_BUS_ADCS_HEARTBEAT_LENGTH];
    can_bus_adcs_heartbeat_pack(data, &hb, sizeof(data));
    can_transmit(CAN_BUS_ADCS_HEARTBEAT_FRAME_ID, data, CAN_BUS_ADCS_HEARTBEAT_LENGTH);
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

static void task_imu_read(void) {
    mpu6050_data_t raw;

    if (mpu6050_read_all(&raw)) {
        imu.accel[0] = raw.accel_x;
        imu.accel[1] = raw.accel_y;
        imu.accel[2] = raw.accel_z;
        imu.gyro[0] = raw.gyro_x;
        imu.gyro[1] = raw.gyro_y;
        imu.gyro[2] = raw.gyro_z;
        imu.temp = raw.temp;
        adcs_state.error_flags &= ~ERR_IMU_FAULT;
    } else {
        adcs_state.error_flags |= ERR_IMU_FAULT;
    }
}

static void task_imu_print(void) {
    uart_print_float(imu.accel[0], 2);
    uart_print_str(",");
    uart_print_float(imu.accel[1], 2);
    uart_print_str(",");
    uart_print_float(imu.accel[2], 2);
    uart_print_str(",");
    uart_print_float(imu.gyro[0], 2);
    uart_print_str(",");
    uart_print_float(imu.gyro[1], 2);
    uart_print_str(",");
    uart_print_float(imu.gyro[2], 2);
    uart_print_str("\r\n");
}

static void task_attitude_update(void) {
    // TODO(acg): Use complementary filter or Kalman filter from ccl
    float dt = IMU_READ_INTERVAL_MS / 1000.0f;
    attitude.roll += imu.gyro[0] * dt;
    attitude.pitch += imu.gyro[1] * dt;
    attitude.yaw += imu.gyro[2] * dt;
}

static void task_motor_control(void) {
    if (adcs_state.mode == ADCS_MODE_IDLE || !motor.cmd_enable) {
        pwm_set_duty(0);
        return;
    }
    
    if (adcs_state.mode == ADCS_MODE_OPEN_LOOP) {
        pwm_set_duty(motor.cmd_duty_cycle);
    } else if (adcs_state.mode == ADCS_MODE_CLOSED_LOOP) {
        // TODO(acg): Use PID controller from ccl
        // float error = motor.cmd_rpm - motor.actual_rpm;
        // float output = pid_update(&motor_pid, error);
        // pwm_set_duty((uint16_t)output);
    }
    
    // TODO: Read encoder
    // motor.actual_rpm = motor_encoder_get_rpm();
    
    // TODO: Read current sensor
}

static void task_attitude_tx(void) {
    struct can_bus_adcs_att_est_t msg = {
        .roll = (int16_t)(attitude.roll * 10),  // Scale for 0.1 factor
        .pitch = (int16_t)(attitude.pitch * 10),
        .yaw = (int16_t)(attitude.yaw * 10)
    };
    
    uint8_t data[CAN_BUS_ADCS_ATT_EST_LENGTH];
    can_bus_adcs_att_est_pack(data, &msg, sizeof(data));
    can_transmit(CAN_BUS_ADCS_ATT_EST_FRAME_ID, data, CAN_BUS_ADCS_ATT_EST_LENGTH);
}

static void task_motor_status_tx(void) {
    struct can_bus_adcs_motor_status_t msg = {
        .rpm = motor.actual_rpm,
        .current = (uint16_t)(motor.current * 100),  // Scale for 0.01 factor
        .fault = motor.fault
    };
    
    uint8_t data[CAN_BUS_ADCS_MOTOR_STATUS_LENGTH];
    can_bus_adcs_motor_status_pack(data, &msg, sizeof(data));
    can_transmit(CAN_BUS_ADCS_MOTOR_STATUS_FRAME_ID, data, CAN_BUS_ADCS_MOTOR_STATUS_LENGTH);
}

static void task_imu_tx(void) {
    struct can_bus_adcs_imu_accel_t accel_msg = {
        .accel_x = (int16_t)(imu.accel[0] * 100),  // Scale for 0.01 factor
        .accel_y = (int16_t)(imu.accel[1] * 100),
        .accel_z = (int16_t)(imu.accel[2] * 100)
    };
    
    uint8_t data[CAN_BUS_ADCS_IMU_ACCEL_LENGTH];
    can_bus_adcs_imu_accel_pack(data, &accel_msg, sizeof(data));
    can_transmit(CAN_BUS_ADCS_IMU_ACCEL_FRAME_ID, data, CAN_BUS_ADCS_IMU_ACCEL_LENGTH);
    
    struct can_bus_adcs_imu_gyro_t gyro_msg = {
        .gyro_x = (int16_t)(imu.gyro[0] * 10),  // Scale for 0.1 factor
        .gyro_y = (int16_t)(imu.gyro[1] * 10),
        .gyro_z = (int16_t)(imu.gyro[2] * 10)
    };
    
    can_bus_adcs_imu_gyro_pack(data, &gyro_msg, sizeof(data));
    can_transmit(CAN_BUS_ADCS_IMU_GYRO_FRAME_ID, data, CAN_BUS_ADCS_IMU_GYRO_LENGTH);
}

static void task_can_rx(void) {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
    
    while (can_receive(&id, data, &len)) {
        switch (id) {
            case CAN_BUS_OBC_HEARTBEAT_FRAME_ID: {
                struct can_bus_obc_heartbeat_t msg;
                can_bus_obc_heartbeat_unpack(&msg, data, len);
                subsystem_health.last_obc_heartbeat = clock_get_ms();
                adcs_state.error_flags &= ~ERR_OBC_TIMEOUT;
                break;
            }
            case CAN_BUS_OBC_MODE_CMD_FRAME_ID: {
                struct can_bus_obc_mode_cmd_t msg;
                can_bus_obc_mode_cmd_unpack(&msg, data, len);
                adcs_state.mode = msg.mode;
                break;
            }
            case CAN_BUS_ADCS_MOTOR_CMD_FRAME_ID: {
                struct can_bus_adcs_motor_cmd_t msg;
                can_bus_adcs_motor_cmd_unpack(&msg, data, len);
                motor.cmd_duty_cycle = msg.duty_cycle;
                motor.cmd_enable = msg.enable;
                break;
            }
        }
    }
}

static void adcs_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (1 << 14) | (1 << 28);

    clock_init();
    SysTick_Init();
    __enable_irq();

    uart_init();
    i2c_init();
    can_init();
    eth_init();
    // pwm_init();
    // motor_encoder_init();

    if (!mpu6050_init()) {
        adcs_state.error_flags |= ERR_IMU_FAULT;
    }

    uint32_t now = clock_get_ms();
    subsystem_health.last_obc_heartbeat = now;
}

int main(void) {
    adcs_init();

    while (1) {
        int32_t now = clock_get_ms();

        // IMU read @ 100 Hz
        if (now - last_run.imu_read >= IMU_READ_INTERVAL_MS) {
            task_imu_read();
            task_attitude_update();
            last_run.imu_read = now;
        }
        
        // Motor control @ 100 Hz
        // if (now - last_run.motor_control >= MOTOR_CONTROL_INTERVAL_MS) {
        //     task_motor_control();
        //     last_run.motor_control = now;
        // }
        
        // Ethernet status @ 10 Hz
        if (now - last_run.eth_check >= ETH_CHECK_INTERVAL_MS) {
            task_eth_status();
            last_run.eth_check = now;
        }

        // Heartbeat @ 1 Hz
        if (now - last_run.heartbeat >= ADCS_HEARTBEAT_INTERVAL_MS) {
            task_heartbeat(now);
            last_run.heartbeat = now;
        }
        
        // Every loop (non-blocking)
        task_can_rx();

        if (now - subsystem_health.last_obc_heartbeat > ADCS_HEARTBEAT_TIMEOUT_MS) {
            adcs_state.error_flags |= ERR_OBC_TIMEOUT;
        }
    }
}
#ifndef ADCS_CONFIG_H
#define ADCS_CONFIG_H

// Timing (ms)
#define ADCS_HEARTBEAT_INTERVAL_MS  1000
#define ADCS_HEARTBEAT_TIMEOUT_MS   3000
#define ETH_CHECK_INTERVAL_MS       100
#define IMU_READ_INTERVAL_MS        10      // 100 Hz
#define MOTOR_CONTROL_INTERVAL_MS   10      // 100 Hz
#define ATTITUDE_TX_INTERVAL_MS     100     // 10 Hz
#define MOTOR_STATUS_TX_INTERVAL_MS 100     // 10 Hz

// Modes
#define ADCS_MODE_IDLE          0
#define ADCS_MODE_OPEN_LOOP     1
#define ADCS_MODE_CLOSED_LOOP   2
#define ADCS_MODE_SAFE          3
#define ADCS_MODE_CRITICAL      4

// Error flags
#define ERR_IMU_FAULT      (1 << 0)
#define ERR_MOTOR_FAULT    (1 << 1)
#define ERR_CAN_FAULT      (1 << 2)
#define ERR_ADCS_TIMEOUT   (1 << 3)
#define ERR_OBC_TIMEOUT    (1 << 4)

#endif
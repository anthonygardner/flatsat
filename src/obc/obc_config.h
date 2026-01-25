#ifndef OBC_CONFIG_H
#define OBC_CONFIG_H

// Timing (ms)
#define OBC_HEARTBEAT_INTERVAL_MS   1000
#define ETH_CHECK_INTERVAL_MS       100
#define OBC_HEARTBEAT_TIMEOUT_MS    3000

// Modes
#define OBC_MODE_IDLE       0
#define OBC_MODE_ACTIVE     1
#define OBC_MODE_SAFE       2
#define OBC_MODE_CRITICAL   3

// Error flags
#define ERR_ADCS_TIMEOUT    (1 << 0)
#define ERR_EPS_TIMEOUT     (1 << 1)
#define ERR_CAN_ERROR       (1 << 2)
#define ERR_ETH_DOWN        (1 << 3)

#endif
#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include <stdint.h>

void can_init(void);

bool can_receive(uint32_t *id, uint8_t *data, uint8_t *len);

bool can_transmit(uint32_t id, uint8_t *data, uint8_t len);

#endif
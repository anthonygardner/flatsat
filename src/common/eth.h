#ifndef ETH_H
#define ETH_H

#include <stdbool.h>
#include <stdint.h>

void eth_init(void);

bool eth_receive(uint8_t *buffer, uint16_t *length);

uint16_t eth_get_link_status(void);

#endif
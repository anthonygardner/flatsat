#ifndef ETH_H
#define ETH_H

#include <stdint.h>

void eth_init(void);

uint16_t eth_get_link_status(void);

#endif
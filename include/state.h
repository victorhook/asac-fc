#ifndef STATE_H
#define STATE_H

#include "stdlib.h"
#include "stdint.h"


typedef struct {
    bool     is_armed;
    uint64_t last_recieved_rx_packet;
} state_t;

extern state_t state;


#endif /* STATE_H */

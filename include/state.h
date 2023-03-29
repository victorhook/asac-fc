#ifndef STATE_H
#define STATE_H

#include "pico/stdlib.h"


typedef enum {
    MODE_BOOTING,
    MODE_IDLE,
    MODE_PANIC
} drone_mode_t;


typedef struct {
    bool     is_armed;
    uint64_t last_recieved_rx_packet;
    drone_mode_t   mode;
} state_t;

extern state_t state;


#endif /* STATE_H */

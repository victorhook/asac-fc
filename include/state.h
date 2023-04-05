#ifndef STATE_H
#define STATE_H

#include "pico/stdlib.h"


typedef enum {
    MODE_BOOTING,
    MODE_IDLE,
    MODE_PANIC,
    MODE_ERROR
} drone_mode_t;


typedef struct {
    bool         is_armed;
    bool         is_connected;
    drone_mode_t mode;
} state_t;

extern state_t state;


#endif /* STATE_H */

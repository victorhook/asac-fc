#include "state.h"


state_t state = {
    .is_armed       = false,
    .is_connected   = false,
    .can_run_motors = false,
    .mode           = MODE_BOOTING,
    .roll           = 0,
    .pitch          = 0,
    .yaw            = 0,
    .velocity_x     = 0,
    .velocity_y     = 0,
    .velocity_z     = 0,
    .pos_x          = 0,
    .pos_y          = 0,
    .pos_z          = 0
};

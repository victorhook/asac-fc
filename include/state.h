#ifndef STATE_H
#define STATE_H

#include "asac_fc.h"


typedef enum {
    MODE_BOOTING,
    MODE_IDLE,
    MODE_PANIC,
    MODE_ERROR
} drone_mode_t;

typedef struct {
    bool         is_armed;
    bool         is_force_armed;
    bool         is_rc_connected;
    bool         is_usb_connected;
    bool         can_run_motors;
    drone_mode_t mode;

    float        roll;
    float        pitch;
    float        yaw;

    float        roll_speed;
    float        pitch_speed;
    float        yaw_speed;

    float        velocity_x;
    float        velocity_y;
    float        velocity_z;

    float        pos_x;
    float        pos_y;
    float        pos_z;
} state_t;

extern state_t state;


#endif /* STATE_H */

#ifndef STATE_H
#define STATE_H

#include "util.h"

typedef enum
{
    FLIGHTMODE_STABILIZE = 0,
    FLIGHTMODE_ACRO = 1
} flightmode_t;


typedef enum {
    MODE_BOOTING,
    MODE_IDLE,
    MODE_PANIC,
    MODE_ERROR
} drone_mode_t;

typedef struct {
    bool armed;
    bool force_armed;
    bool rc_connected;
    bool usb_connected;
    bool run_motor_test;    
    bool imu_is_working;

    drone_mode_t mode;
    flightmode_t flightmode;

    uint16_t bat_volt_mv;
    int16_t bat_curr_ma;
    int8_t bat_remain;

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

bool allow_rebooting();

int arming_check();


#endif /* STATE_H */

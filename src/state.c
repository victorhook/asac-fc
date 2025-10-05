#include "state.h"


state_t state = {
    .armed         = false,
    .force_armed   = false,
    .rc_connected  = false,
    .usb_connected = false,
    .imu_is_working   = false,
    .run_motor_test   = false,
    .mode             = MODE_BOOTING,
    .flightmode       = FLIGHTMODE_ACRO,
    .bat_volt_mv      = 0,
    .bat_curr_ma      = -1,
    .bat_remain       = -1,
    .roll             = 0,
    .pitch            = 0,
    .yaw              = 0,
    .velocity_x       = 0,
    .velocity_y       = 0,
    .velocity_z       = 0,
    .pos_x            = 0,
    .pos_y            = 0,
    .pos_z            = 0
};


bool allow_rebooting()
{
    return !state.armed && !state.force_armed;
}

int arming_check()
{
    return
        (state.rc_connected)   &&
        (state.imu_is_working) &&
        (!state.usb_connected);
}

#include "pid_controller.h"
#include "pid.h"

#include "pico/stdlib.h"


static pid_state_t pid_roll;
static pid_state_t pid_pitch;
static pid_state_t pid_yaw;
static uint32_t last_update;


int pid_controller_init() {
    last_update = time_us_32();

    memset(&pid_roll, 0, sizeof(pid_state_t));
    pid_roll.Kp = 1;
    pid_roll.Ki = 0;
    pid_roll.Kd = 0;

    memset(&pid_pitch, 0, sizeof(pid_state_t));
    pid_pitch.Kp = 1;
    pid_pitch.Ki = 0;
    pid_pitch.Kd = 0;

    memset(&pid_yaw, 0, sizeof(pid_state_t));
    pid_yaw.Kp = 1;
    pid_yaw.Ki = 0;
    pid_yaw.Kd = 0;

    return 0;
}


void pid_controller_update(const rates_t* measured, const rates_t* desired, pid_adjust_t* adjust) {
    uint32_t dt_s = (time_us_32() - last_update) / 1000000.0;

    adjust->roll  = pid_update(&pid_roll, measured->roll, desired->roll, dt_s);
    adjust->pitch = pid_update(&pid_pitch, measured->pitch, desired->pitch, dt_s);
    adjust->yaw   = pid_update(&pid_yaw, measured->yaw, desired->yaw, dt_s);

    last_update = time_us_32();
}

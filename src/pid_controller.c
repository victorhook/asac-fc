#include "pid_controller.h"
#include "pid.h"

static pid_t pid_roll;
static pid_t pid_pitch;
static pid_t pid_yaw;
static uint64_t last_update;


void pid_controller_init() {
    last_update = getCurrentTime();
}


void pid_controller_update(const rates_t* measured, const rates_t* desired, pid_adjust_t* adjust) {
    uint32_t dt_s = (getCurrentTime() - last_update) / 1000.0;

    adjust->roll  = pid_update(&pid_roll, measured->roll, desired->roll, dt);
    adjust->pitch = pid_update(&pid_pitch, measured->pitch, desired->pitch, dt);
    adjust->yaw   = pid_update(&pid_yaw, measured->yaw, desired->yaw, dt);

    last_update = getCurrentTime();
}

#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include "pid/pid.h"
#include "util.h"

int atrc_controller_init();

void atrc_controller_update(const vec3f_t* desired, const vec3f_t* measured, const float throttle, const float dt_s);

void atcr_controller_reset();


extern pid_handle_t pid_roll;
extern pid_handle_t pid_pitch;
extern pid_handle_t pid_yaw;


#endif
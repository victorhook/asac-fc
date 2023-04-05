#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "pid.h"


int pid_controller_init();


void pid_controller_update(const rates_t* measured, const rates_t* desired, pid_adjust_t* pid_adjust);

/*
 * Reset the all PID variables to their default values.
 */
void pid_controller_reset();

// Global -- TODO: Better way?
extern pid_state_t pid_roll;
extern pid_state_t pid_pitch;
extern pid_state_t pid_yaw;


#endif /* PID_CONTROLLER_H */

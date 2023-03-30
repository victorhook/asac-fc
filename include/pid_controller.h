#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "pid.h"


int pid_controller_init();


void pid_controller_update(const rates_t* measured, const rates_t* desired, pid_adjust_t* pid_adjust);


#endif /* PID_CONTROLLER_H */

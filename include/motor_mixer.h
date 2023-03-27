#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include "pid.h"


typedef struct {

} motor_command_t;


void motor_mixer_update(const pid_adjust_t* adjust, motor_command_t* motor_command);


void set_motor_speeds(const motor_command_t* motor_command);


#endif /* MOTOR_MIXER_H */

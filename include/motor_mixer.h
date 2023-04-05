#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include "pid.h"
#include "stdint.h"

typedef struct {
    float m1;
    float m2;
    float m3;
    float m4;
}__attribute__((packed)) motor_mixer_command_t;


void motor_mixer_update(const uint16_t throttle, const pid_adjust_t* adjust, motor_mixer_command_t* motor_command);


void set_motor_speeds(const motor_mixer_command_t* motor_command);


#endif /* MOTOR_MIXER_H */

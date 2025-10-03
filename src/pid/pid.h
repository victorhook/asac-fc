#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Kff;
    float imax;
    float error;
    float prev_error;
    float d_err;
    float p;
    float i;
    float d;
    float ff;
    float out;
} pid_t;

void pid_reset(pid_t* pid);

float pid_update(pid_t* pid, const float target, const float actual, const bool skip_integrator, const float dt);


#endif /* PID_H */

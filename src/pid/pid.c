#include "pid.h"
#include "hal.h" 


void pid_reset(pid_t* pid)
{
    pid->error = 0;
    pid->prev_error = 0;
    pid->d_err = 0;
    pid->p = 0;
    pid->i = 0;
    pid->d = 0;
    pid->ff = 0;
    pid->out = 0;
}

float pid_update(pid_t* pid, const float target, const float actual, const bool skip_integrator, const float dt)
{
    pid->error = target - actual;

    // P
    pid->p = pid->error * pid->Kp;

    // I
    if (skip_integrator)
    {
        pid->i = 0;
    }
    else
    {
        pid->i += pid->error * pid->Ki * dt;
        // Simple anti-windup by limiting sum
        pid->i = constrainf(pid->i, -pid->imax, pid->imax);
    }

    // D
    pid->d_err = pid->error - pid->prev_error;
    pid->d = pid->d_err * pid->Kd / dt;

    // Feed forward
    pid->ff = pid->Kff * target;    

    // Summarize all parts
    pid->out = pid->p + pid->i + pid->d + pid->ff;

    // Update previous error for next run
    pid->prev_error = pid->error;

    return pid->out;
}

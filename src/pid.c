#include "pid.h"


float pid_update(pid_state_t* pid, const float measured, const float desired, float dt_s) {
    // Calculate error
    float error = desired - measured;

    // Calculate difference with error
    float dErr = (error - pid->last_err) * dt_s;

    // Update integral sum
    pid->err_integral += error * dt_s;

    float p = pid->Kp * error;
    float i = pid->Ki * pid->err_integral;
    float d = pid->Kd * dErr;

    float result = p + i + d;

    pid->last_err = error;

    return result;
}
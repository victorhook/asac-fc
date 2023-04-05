#include "pid.h"
#include "stdio.h"


#define INTERGRAL_LIMIT_BOTTOM
#define INTERGRAL_LIMIT_TOP


float pid_update(pid_state_t* pid, const float measured, const float desired, float dt_s) {
    // Calculate error
    float error = desired - measured;

    // Calculate difference with error
    float dErr = (error - pid->last_err) / dt_s;

    // Update integral sum
    pid->err_integral += error * dt_s;

    // Check if we need to disable I-term, anti-windup
    if (!pid->integral_disabled && pid->err_integral > pid->integral_limit_threshold) {
        // Anti-windup START
        pid->integral_disabled = true;
        pid->integral_disabled_timestamp = time_us_32();
    } else if (pid->integral_disabled && pid->err_integral < pid->integral_limit_threshold) {
        // Anti-windup STOP
        pid->integral_disabled = false;
    }

    float p;
    float i;
    float d;

    p = pid->Kp * error;
    if (pid->integral_disabled) {
        i = 0;
    } else {
        i = pid->Ki * pid->err_integral;
    }
    d = pid->Kd * dErr;

    float result = p + i + d;

    pid->last_err = error;

    return result;
}
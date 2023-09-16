#include "pid.h"


float pid_update(pid_state_t* pid, const float measured, const float desired, const uint16_t throttle, const float dt_s) {
    // Calculate error
    pid->err = desired - measured;

    // Calculate difference with error
    pid->d_err = (pid->err - pid->last_err) / dt_s;

    // If the throttle is very low, we probably don't want the integrator
    // to run. Image holding the drone and making full pitch, which would
    // cause the integral to increase.
    //
    // Input throttle is 1000-2000, so the limit of ~1100-1200 seems ok
    if (throttle < 1175) {
        pid->integral_disabled = true;
    } else {
        // Update integral sum
        pid->err_integral += pid->err * dt_s;

        if (pid->integral_disabled) {
            if ((pid->err_integral > -pid->integral_limit_threshold) &&
                (pid->err_integral < pid->integral_limit_threshold)) {
                // Anti-windup STOP
                pid->integral_disabled = false;
            }
        } else {
            // Check if we need to disable I-term, anti-windup
            if ((pid->err_integral > pid->integral_limit_threshold) ||
                (pid->err_integral < -pid->integral_limit_threshold)) {
                // Anti-windup START
                pid->integral_disabled = true;
                pid->integral_disabled_timestamp = time_us_32();
            }
        }
    }

    // Cap error integral sum to windup limit
    pid->err_integral = constrain(pid->err_integral, -pid->integral_limit_threshold, pid->integral_limit_threshold);

    pid->p = pid->Kp * pid->err;

    if (pid->integral_disabled) {
        pid->i = 0;
    } else {
        pid->i = pid->Ki * pid->err_integral;
    }

    // Feed forward
    pid->ff = pid->Kff * desired;

    pid->d = pid->Kd * pid->d_err;

    pid->last_err = pid->err;

    pid->pid = pid->p + pid->i + pid->d + pid->ff;

    return pid->pid;
}

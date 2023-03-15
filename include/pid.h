#ifndef PID_H
#define PID_H

#include "stdint.h"

#define getCurrentTime() 0

typedef struct {
    uint16_t Kp;
    uint16_t Ki;
    uint16_t Kd;
    uint16_t feed_forward;
    float err_integral;
    float last_err;
} pid_t;


// Rotation rates in degrees per second (deg/s)
typedef struct {
    float roll;
    float pitch;
    float yaw;
} rates_t;


// The pid adjust are used to know how much to adjust each motor speed, depending
// on the pid parameter values, and the error. This is the result of a pid update loop.
typedef struct {
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
} pid_adjust_t;


float pid_update(pid_t* pid, const float measured, const float desired, float dt_s)


#endif /* PID_H */

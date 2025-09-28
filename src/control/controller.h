#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "util.h"
#include "imu/imu.h"
#include "motor/motor.h"
#include "rc/rc.h"
#include "pid/pid.h"
#include "battery/battery_adc.h"


typedef struct {
    uint16_t throttle;
    rates_t rates;
} setpoint_t;


int controller_init();

void controller_set_motors();

void controller_telemetry();

void controller_debug();

void controller_pid_loop();

#endif /* CONTROLLER_H */

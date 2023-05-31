#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "asac_fc.h"
#include "imu.h"
#include "motor.h"
#include "receiver.h"
#include "pid.h"


typedef struct {
    uint16_t throttle;
    rates_t rates;
} setpoint_t;


int controller_init();

void controller_set_motors();

void controller_debug();

void controller_update();


// Intermediate variables used to calculate correct commands for motors
//   from the RC input.
// These purpose of having variables for each step in the control loop
//   is to make debugging and logging easier.
extern imu_reading_t         imu_reading;
extern imu_reading_t         imu_bias;
extern imu_reading_t         imu_filtered;
extern rates_t               ctrl_attitude_rates_measured;
extern rc_input_t            ctrl_rc_input_raw;
extern rc_input_t            ctrl_rc_input_constrained;
extern setpoint_t            setpoint;
extern pid_adjust_t          ctrl_attitude_rates_adjust; // @cal;
extern motor_command_t       ctrl_motor_mixer_command;
extern motor_command_t       ctrl_motor_command_non_restricted;
extern motor_command_t       ctrl_motor_command;
extern uint32_t              last_ctrl_update;
extern bool                  can_run_motors;
extern pid_state_t           pid_roll;
extern pid_state_t           pid_pitch;
extern pid_state_t           pid_yaw;



#endif /* CONTROLLER_H */

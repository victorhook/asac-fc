#include "pid_controller.h"
//#include "imu.h"
#include "receiver.h"

static rates_t state_attitude_rates_measured;
static rates_t state_attitude_rates_desired;
static pid_adjust_t state_attitude_rates_adjust;
static rc_input_t state_rc_input;
static motor_commands_t state_motor_commands;

void controller_update() {
    imu_get_latest_reading(&state_attitude_rates_measured);

    receiver_get_latest_reading(&state_rc_input);

    convert_rc_input_to_desired_rates(&state_rc_input, &state_attitude_rates_desired);

    pid_controller_update(&state_attitude_rates_measured, &state_attitude_rates_desired, &state_attitude_rates_adjust);

    motor_mixer_update(&state_attitude_rates_adjust, &state_motor_commands);

    set_motor_speeds(&state_motor_commands);
}

static void convert_rc_input_to_desired_rates(const rc_input_t* state_rc_input, rates_t* state_attitude_rates_desired) {

}
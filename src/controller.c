#include "pid_controller.h"
#include "pid.h"
#include "imu.h"
#include "receiver.h"
#include "motor_mixer.h"
#include "motor.h"

static imu_reading_t imu_reading;
static rates_t state_attitude_rates_measured;
static rates_t state_attitude_rates_desired;
static pid_adjust_t state_attitude_rates_adjust; // @cal;
static rc_input_t state_rc_input;
static motor_command_t state_motor_command;

static void convert_rc_input_to_desired_rates(const rc_input_t* state_rc_input, rates_t* state_attitude_rates_desired);

void controller_init() {

}

void controller_update() {
    /*
    imu_get_latest_reading(&imu_reading);
    //state_attitude_rates_measured

    receiver_get_last_packet(&state_rc_input);

    convert_rc_input_to_desired_rates(&state_rc_input, &state_attitude_rates_desired);

    pid_controller_update(&state_attitude_rates_measured, &state_attitude_rates_desired, &state_attitude_rates_adjust);

    motor_mixer_update(&state_attitude_rates_adjust, &state_motor_command);

    set_motor_speeds(&state_motor_command);
    */
}

static void convert_rc_input_to_desired_rates(const rc_input_t* state_rc_input, rates_t* state_attitude_rates_desired) {

}
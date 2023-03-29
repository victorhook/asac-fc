#include "pid_controller.h"
#include "pid.h"
#include "imu.h"
#include "receiver.h"
#include "motor_mixer.h"
#include "motor.h"

#include "pico/stdlib.h"

static imu_reading_t imu_reading;
static rates_t state_attitude_rates_measured;
static rates_t state_attitude_rates_desired;
static pid_adjust_t state_attitude_rates_adjust; // @cal;
static rc_input_t state_rc_input;
static motor_command_t state_motor_command;
static uint64_t last_update;

static void convert_rc_input_to_desired_rates(const rc_input_t* state_rc_input, rates_t* state_attitude_rates_desired);

int controller_init() {
    last_update = time_us_64();
    return 0;
}

void controller_update() {
    printf("CTRL\n");

    // Step X. Read data from IMU
    imu_read(&imu_reading);

    // Step X. Filter IMU data?

    // Step X. IMU reading to attitude rates
    state_attitude_rates_measured.roll  = imu_reading.gyro_x;
    state_attitude_rates_measured.pitch = imu_reading.gyro_y;
    state_attitude_rates_measured.yaw   = imu_reading.gyro_z;

    // Step X. Get latest data from receiver
    receiver_get_last_packet(&state_rc_input);

    // Step X. Map receiver data to desired rotation rates.
    convert_rc_input_to_desired_rates(&state_rc_input, &state_attitude_rates_desired);

    // Step X. Update PID values with IMU reading and desired rotation rates.
    pid_controller_update(&state_attitude_rates_measured, &state_attitude_rates_desired, &state_attitude_rates_adjust);

    // Step X. Pass pid values to motor mixer to get commands for motors.
    motor_mixer_update(&state_attitude_rates_adjust, &state_motor_command);

    // Step X. Set motor speeds
    //set_motor_speeds(&state_motor_command);
}

static void convert_rc_input_to_desired_rates(const rc_input_t* state_rc_input, rates_t* state_attitude_rates_desired) {

}
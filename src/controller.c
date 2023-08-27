#include "controller.h"
#include "state.h"
#include "led.h"
#include "telemetry.h"
#include "log.h"
#include "math.h"
#include "mavlink_params.h"


// RC Input mappings, should they be here?
#define RC_CHANNEL_THROTTLE 2
#define RC_CHANNEL_ROLL 0
#define RC_CHANNEL_PITCH 1
#define RC_CHANNEL_YAW 3
#define RC_CHANNEL_ARM 4

typedef struct {
    uint16_t min;
    uint16_t max;
} rc_channel_range_t;


// If between range, we'll arm
rc_channel_range_t arm_range = {
    .min = 1500,
    .max = 2000,
};

#define THROTTLE_MAX 2000
#define THROTTLE_MIN 1175

#define RATES_ROLL_MAX  400
#define RATES_PITCH_MAX 400
#define RATES_YAW_MAX   400

// If no packed received within this timeout, we consider ourself NOT connected.
#define CONNECTED_TIMEOUT_MS 500


// Intermediate variables used to calculate correct commands for motors
//   from the RC input.
// These purpose of having variables for each step in the control loop
//   is to make debugging and logging easier.
imu_reading_t         imu_raw;
imu_reading_t         imu_bias;
imu_reading_t         imu_no_bias;
imu_reading_t         imu_filtered;
imu_reading_t         imu_filtered_dterm;
rates_t               attitude_rates_measured;
rx_state_t            rx_state;
rc_input_t            ctrl_rc_input_raw;
rc_input_t            ctrl_rc_input_constrained;
setpoint_t            setpoint;
pid_adjust_t          attitude_rates_adjust; // @cal;
motor_command_t       motor_mixer_command;
motor_command_t       motor_command_non_restricted;
motor_command_t       ctrl_motor_command;
uint32_t              last_ctrl_update;
uint32_t              last_pid_update;
pid_state_t           pid_roll;
pid_state_t           pid_pitch;
pid_state_t           pid_yaw;


#ifdef TELEMETRY_LOGGING
    static log_block_data_control_loop_t log_block;
    static void telemetry_log();
#endif

static void remove_bias_from_imu_reading(imu_reading_t* imu_no_bias, const imu_reading_t* imu_raw, const imu_reading_t* imu_bias);

static void constrain_rc_input(const rc_input_t* unconstrained, rc_input_t* constrained);

static void convert_rc_input_to_setpoint(const rc_input_t* state_rc_input, setpoint_t* setpoint);

static void state_estimator(const imu_reading_t* imu_filtered, const float ctrl_loop_dt_s, state_t* state);

static bool is_armed(const rc_input_t* rc_input_constrained);

static bool is_connected(const rc_input_t* rc_input_raw);

static void disconnect();

static void connect();

static void arm();

static void disarm();

static void motor_mixer_update(uint16_t throttle, const pid_adjust_t* adjust, motor_command_t* motor_command);

static int pid_controller_init();

static void pid_controller_update(const rates_t* measured, const setpoint_t* desired, pid_adjust_t* pid_adjust);

static void pid_controller_reset();

// Print debug
static void print_rc_input();


int controller_init() {
    last_ctrl_update = us_since_boot();

    // Note that the controller is expected to be initialized AFTER IMU.
    // This is so that we can retrieve the correct IMU offsets/bias.
    const imu_reading_t* imu_bias_ = imu_get_bias();
    memcpy(&imu_bias, imu_bias_, sizeof(imu_reading_t));

    int result = pid_controller_init();

    return result;
}

void controller_update() {
    // Average controller update time: ~300us
    // Measured experimentally

    uint32_t ctrl_loop_started = us_since_boot();
    float ctrl_loop_dt_s = (float) (ctrl_loop_started - last_ctrl_update) / 1000000.0;

    // Read data from IMU
    imu_read(&imu_raw);

    // Ensure the orientation of the coordinate is correct
    imu_raw.gyro_x *= IMU_ORIENTATION_X;
    imu_raw.gyro_y *= IMU_ORIENTATION_Y;
    imu_raw.gyro_z *= IMU_ORIENTATION_Z;

    // Remove bias from imu readings
    remove_bias_from_imu_reading(&imu_no_bias, &imu_raw, &imu_bias);

    // Filter gyro readings
    // TODO: Filter D term?
    imu_filter_gyro((vector_3d_t*) &imu_filtered.gyro_x, (vector_3d_t*) &imu_no_bias.gyro_x);

    // IMU reading to attitude rates
    attitude_rates_measured.roll  = imu_filtered.gyro_x;
    attitude_rates_measured.pitch = imu_filtered.gyro_y;
    attitude_rates_measured.yaw   = imu_filtered.gyro_z;

    // Give measurements to state estimator, that estimates our current state
    state_estimator(&imu_filtered, ctrl_loop_dt_s, &state);

    // Get latest state from the receiver
    receiver_get_state(&rx_state);
    rc_input_t* ctrl_rc_input_raw = &rx_state.last_packet;

    // Check if we're connected (gotten radio packet within ~X ms)
    bool connected = is_connected(ctrl_rc_input_raw);
    if (connected != state.is_connected) {
        if (connected) {
            connect();
        } else {
            disconnect();
        }
    }

    if (state.is_connected) {
        // Constrain/crop RC input in case they're out of expected range.
        constrain_rc_input(ctrl_rc_input_raw, &ctrl_rc_input_constrained);

        // Map receiver data to desired rotation rates.
        convert_rc_input_to_setpoint(&ctrl_rc_input_constrained, &setpoint);
    }

    // Check if we're armed
    bool armed = is_armed(&ctrl_rc_input_constrained);
    if (armed != state.is_armed) {
        if (armed) {
            arm();
        } else {
            disarm();
        }
    }

    // Safety mechanism for running motors.
    // TODO: Different handling of this with flags, eg connected to USB.
    if (state.is_armed && state.is_connected) {
        state.can_run_motors = true;
    } else {
        state.can_run_motors = false;
    }

    // If we cannot run the motors, we set setpoint to 0 for all
    // TODO: Is his necessary, since some lines below, we set the motor commands
    // to 0 if we cannot run the motors anyways.
    if (!state.can_run_motors) {
        setpoint.rates.roll  = 0;
        setpoint.rates.pitch = 0;
        setpoint.rates.yaw   = 0;
        setpoint.throttle    = 0;
    }

    // Update PID values with IMU reading and desired rotation rates.
    pid_controller_update(&attitude_rates_measured, &setpoint, &attitude_rates_adjust);

    // Pass PID values to motor mixer to get commands for motors.
    motor_mixer_update(setpoint.throttle, &attitude_rates_adjust, &motor_mixer_command);

    // Map motor commands to values between THROTTLE_MIN and THROTTLE_MAX,
    // then convert to float between 0-1.
    // This is because the motor controller wants a value between 0-1.
    motor_command_non_restricted.m1 = (constrain(motor_mixer_command.m1, THROTTLE_MIN, THROTTLE_MAX) - 1000) / 1000.0;
    motor_command_non_restricted.m2 = (constrain(motor_mixer_command.m2, THROTTLE_MIN, THROTTLE_MAX) - 1000) / 1000.0;
    motor_command_non_restricted.m3 = (constrain(motor_mixer_command.m3, THROTTLE_MIN, THROTTLE_MAX) - 1000) / 1000.0;
    motor_command_non_restricted.m4 = (constrain(motor_mixer_command.m4, THROTTLE_MIN, THROTTLE_MAX) - 1000) / 1000.0;

    if (state.can_run_motors) {
        ctrl_motor_command.m1 = motor_command_non_restricted.m1;
        ctrl_motor_command.m2 = motor_command_non_restricted.m2;
        ctrl_motor_command.m3 = motor_command_non_restricted.m3;
        ctrl_motor_command.m4 = motor_command_non_restricted.m4;
    } else {
        ctrl_motor_command.m1 = 0;
        ctrl_motor_command.m2 = 0;
        ctrl_motor_command.m3 = 0;
        ctrl_motor_command.m4 = 0;
    }

    // Set motor output
    set_all_motors_pwm(&ctrl_motor_command);

    // Update timestamp with last update
    last_ctrl_update = us_since_boot();
}



// -- Helper functions -- //
void controller_debug() {
    print_rc_input();
    return;
    printf("gx: %.4f, gy: %.4f, gz: %.4f\n",
        pid_roll.Kp,
        pid_roll.Ki,
        pid_roll.Kd
    );
    printf("\n");
    fflush(stdout);
    return;
    printf("%d, %f, %f, %f  |  %.3f, %.3f, %.3f, %.3f\n",
        setpoint.throttle, setpoint.rates.roll, setpoint.rates.pitch, setpoint.rates.yaw,
        motor_mixer_command.m1, motor_mixer_command.m2, motor_mixer_command.m3, motor_mixer_command.m4
        );
    fflush(stdout);
    return;
}

static void print_rc_input() {
    printf("T: %u, ", rx_state.last_packet.timestamp);
    printf("Channels: ");
    for (int i = 0; i < RC_MAX_NBR_OF_CHANNELS; i++) {
        printf("%d ", rx_state.last_packet.channels[i]);
    }
    printf("\n");
}


static void state_estimator(const imu_reading_t* imu_filtered, const float ctrl_loop_dt_s, state_t* state) {
    // Resources:
    // https://www.youtube.com/watch?v=CHSYgLfhwUo&ab_channel=Code%26Supply
    // https://ahrs.readthedocs.io/en/latest/filters/tilt.html

    // Attitude estimation using complementary filter
    const float GYRO_PART = 0.995;
    const float ACCEL_PART = 1 - GYRO_PART;

    float ax = imu_filtered->acc_x;
    float ay = imu_filtered->acc_y;
    float az = imu_filtered->acc_z;

    float gx = imu_filtered->gyro_x;
    float gy = imu_filtered->gyro_y;
    float gz = imu_filtered->gyro_z;

    float acc_roll = atan2f(ay, az);
    float acc_pitch = atan2f(-ax, sqrtf(powf(ay, 2) + powf(az, 2)));

    state->roll = GYRO_PART  * (state->roll + (gx * ctrl_loop_dt_s)) +
                  ACCEL_PART * acc_roll;
    state->pitch = GYRO_PART  * (state->pitch + (gy * ctrl_loop_dt_s)) +
                  ACCEL_PART * acc_pitch;

    state->roll_speed = imu_filtered->gyro_x;
    state->pitch_speed = imu_filtered->gyro_y;
    state->yaw_speed = imu_filtered->gyro_z;
}

static void disconnect() {
    led_set(LED_GREEN, 0);
    state.is_connected = false;
}

static void connect() {
    led_set(LED_GREEN, 1);
    state.is_connected = true;
}

static void arm() {
    led_set(LED_RED, 1);
    state.is_armed = true;
}

static void disarm() {
    led_set(LED_RED, 0);
    state.is_armed = false;
    pid_controller_reset();
}

static void remove_bias_from_imu_reading(imu_reading_t* imu_no_bias, const imu_reading_t* imu_raw, const imu_reading_t* imu_bias) {
    imu_no_bias->acc_x  = imu_raw->acc_x  - imu_bias->acc_x;
    imu_no_bias->acc_y  = imu_raw->acc_y  - imu_bias->acc_y;
    imu_no_bias->acc_z  = imu_raw->acc_z  - imu_bias->acc_z;
    imu_no_bias->gyro_x = imu_raw->gyro_x - imu_bias->gyro_x;
    imu_no_bias->gyro_y = imu_raw->gyro_y - imu_bias->gyro_y;
    imu_no_bias->gyro_z = imu_raw->gyro_z - imu_bias->gyro_z;
}

static bool is_connected(const rc_input_t* rc_input_raw) {
    return ((ms_since_boot() - rc_input_raw->timestamp) < (CONNECTED_TIMEOUT_MS));
}

static bool is_armed(const rc_input_t* rc_input_constrained) {
    uint16_t arm_value = ctrl_rc_input_constrained.channels[RC_CHANNEL_ARM];
    return (arm_value >= arm_range.min) && (arm_value <= arm_range.max);
}

static void constrain_rc_input(const rc_input_t* unconstrained, rc_input_t* constrained) {
    for (int i = 0; i < RC_MAX_NBR_OF_CHANNELS ; i++) {
        constrained->channels[i] = constrain(unconstrained->channels[i], 1000, 2000);
    }
}

static void convert_rc_input_to_setpoint(const rc_input_t* rc_input, setpoint_t* setpoint) {
    int16_t roll     = rc_input->channels[RC_CHANNEL_ROLL];
    int16_t pitch    = rc_input->channels[RC_CHANNEL_PITCH];
    int16_t yaw      = rc_input->channels[RC_CHANNEL_YAW];
    int16_t throttle = rc_input->channels[RC_CHANNEL_THROTTLE];

    // Then we'll map the input to ouput
    // Attitude rates are between:
    //    -MAX  <->  +MAX
    // Eg, MAX = 250 deg/s -> Range: -250 to 250
    setpoint->rates.roll  = -RATES_ROLL_MAX  + ( ( (roll  - 1000) / 1000.0 ) * 2*RATES_ROLL_MAX );
    setpoint->rates.pitch = -RATES_PITCH_MAX + ( ( (pitch - 1000) / 1000.0 ) * 2*RATES_PITCH_MAX );
    setpoint->rates.yaw   = -RATES_YAW_MAX  +  ( ( (yaw   - 1000) / 1000.0 ) * 2*RATES_YAW_MAX );
    setpoint->throttle = throttle;
}

void motor_mixer_update(uint16_t throttle, const pid_adjust_t* adjust, motor_command_t* motor_command) {
    /*
        M4   M2
          \ /
          / \
        M3   M1

        M1 1, -1,  1, -1  <- Rear right
        M2 1, -1, -1,  1  <- Front right
        M3 1,  1,  1,  1  <- Rear left
        M4 1,  1, -1, -1  <- Front left
    */

    if (throttle < THROTTLE_MIN) {
        throttle = THROTTLE_MIN;
    }
    motor_command->m1 = throttle - adjust->roll + adjust->pitch - adjust->yaw;
    motor_command->m2 = throttle - adjust->roll - adjust->pitch + adjust->yaw;
    motor_command->m3 = throttle + adjust->roll + adjust->pitch + adjust->yaw;
    motor_command->m4 = throttle + adjust->roll - adjust->pitch - adjust->yaw;
}

int pid_controller_init() {
    last_pid_update = us_since_boot();

    memset(&pid_roll, 0, sizeof(pid_state_t));
    pid_roll.Kp = system_params.pid_gyro_roll_p.param_value;
    pid_roll.Ki = system_params.pid_gyro_roll_i.param_value;
    pid_roll.Kd = system_params.pid_gyro_roll_d.param_value;
    pid_roll.integral_limit_threshold = 1000;

    memset(&pid_pitch, 0, sizeof(pid_state_t));
    pid_pitch.Kp = system_params.pid_gyro_pitch_p.param_value;
    pid_pitch.Ki = system_params.pid_gyro_pitch_i.param_value;
    pid_pitch.Kd = system_params.pid_gyro_pitch_d.param_value;
    pid_pitch.integral_limit_threshold = 1000;

    memset(&pid_yaw, 0, sizeof(pid_state_t));
    pid_yaw.Kp = system_params.pid_gyro_yaw_p.param_value;
    pid_yaw.Ki = system_params.pid_gyro_yaw_i.param_value;
    pid_yaw.Kd = system_params.pid_gyro_yaw_d.param_value;
    pid_yaw.integral_limit_threshold = 1000;

    return 0;
}

void pid_controller_update(const rates_t* measured, const setpoint_t* desired, pid_adjust_t* adjust) {
    float dt_s = (float) (us_since_boot() - last_pid_update) / 1000000.0;
    adjust->roll  = pid_update(&pid_roll,  measured->roll,  desired->rates.roll,  desired->throttle, dt_s);
    adjust->pitch = pid_update(&pid_pitch, measured->pitch, desired->rates.pitch, desired->throttle, dt_s);
    adjust->yaw   = pid_update(&pid_yaw,   measured->yaw,   desired->rates.yaw,   desired->throttle, dt_s);
    last_pid_update = us_since_boot();
}

void pid_controller_reset() {
    // TODO: Better way of this?
    pid_controller_init();
}


#ifdef TELEMETRY_LOGGING
    static void telemetry_log() {
        log_block.raw_gyro_x = imu_raw.gyro_x;
        log_block.raw_gyro_y = imu_raw.gyro_y;
        log_block.raw_gyro_z = imu_raw.gyro_z;
        log_block.filtered_gyro_x = imu_filtered.gyro_x;
        log_block.filtered_gyro_y = imu_filtered.gyro_y;
        log_block.filtered_gyro_z = imu_filtered.gyro_z;
        log_block.rc_in_roll = ctrl_rc_input_raw.channels[RC_CHANNEL_ROLL];
        log_block.rc_in_pitch = ctrl_rc_input_raw.channels[RC_CHANNEL_PITCH];
        log_block.rc_in_yaw = ctrl_rc_input_raw.channels[RC_CHANNEL_YAW];
        log_block.rc_in_throttle = ctrl_rc_input_raw.channels[RC_CHANNEL_THROTTLE];
        log_block.setpoint_roll = setpoint.rates.roll;
        log_block.setpoint_pitch = setpoint.rates.pitch;
        log_block.setpoint_yaw = setpoint.rates.yaw;
        log_block.setpoint_throttle = setpoint.throttle;
        log_block.is_connected = state.is_connected;
        log_block.is_armed = state.is_armed;
        log_block.can_run_motors = state.can_run_motors;

        // PID params for Roll, Pitch & Yaw:
        log_block.roll_error = pid_roll.err;
        log_block.roll_error_integral = pid_roll.err_integral;
        log_block.roll_p = pid_roll.p;
        log_block.roll_i = pid_roll.i;
        log_block.roll_d = pid_roll.d;
        log_block.roll_pid = pid_roll.pid;

        log_block.pitch_error = pid_pitch.err;
        log_block.pitch_error_integral = pid_pitch.err_integral;
        log_block.pitch_p = pid_pitch.p;
        log_block.pitch_i = pid_pitch.i;
        log_block.pitch_d = pid_pitch.d;
        log_block.pitch_pid = pid_pitch.pid;

        log_block.yaw_error = pid_yaw.err;
        log_block.yaw_error_integral = pid_yaw.err_integral;
        log_block.yaw_p = pid_yaw.p;
        log_block.yaw_i = pid_yaw.i;
        log_block.yaw_d = pid_yaw.d;
        log_block.yaw_pid = pid_yaw.pid;

        log_block.m1_non_restricted = motor_command_non_restricted.m1;
        log_block.m2_non_restricted = motor_command_non_restricted.m2;
        log_block.m3_non_restricted = motor_command_non_restricted.m3;
        log_block.m4_non_restricted = motor_command_non_restricted.m4;

        log_block.m1_restricted = ctrl_motor_command.m1;
        log_block.m2_restricted = ctrl_motor_command.m2;
        log_block.m3_restricted = ctrl_motor_command.m3;
        log_block.m4_restricted = ctrl_motor_command.m4;

        log_block.battery = 0;

        // Radio
        log_block.successful_packets   = radio_stats.successful_packets;
        log_block.parse_errors         = radio_stats.parse_errors;
        log_block.last_received_packet = radio_stats.last_received_packet;
        log_block.packet_rate          = radio_stats.packet_rate;

        telemetry_send_state((log_block_data_t*) &log_block, LOG_TYPE_PID);
    }
#endif
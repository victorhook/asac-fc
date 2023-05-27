#include "controller.h"

#include "asac_fc.h"
#include "pid.h"
#include "state.h"
#include "led.h"
#include "telemetry.h"
#include "log.h"

#include "string.h"
#include "pico/multicore.h"


#define RC_NBR_OF_CHANNELS 14

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

#define RATES_ROLL_MIN  0
#define RATES_ROLL_MAX  100
#define RATES_PITCH_MIN 0
#define RATES_PITCH_MAX 100
#define RATES_YAW_MIN   0
#define RATES_ROLL_MAX  100


// Intermediate variables used to calculate correct commands for motors
//   from the RC input.
// These purpose of having variables for each step in the control loop
//   is to make debugging and logging easier.
imu_reading_t         imu_raw;
imu_reading_t         imu_bias;
imu_reading_t         imu_no_bias;
imu_reading_t         imu_filtered;
rates_t               ctrl_attitude_rates_measured;
rc_input_t            ctrl_rc_input_raw;
rc_input_t            ctrl_rc_input_constrained;
setpoint_t            setpoint;
pid_adjust_t          ctrl_attitude_rates_adjust; // @cal;
motor_mixer_command_t ctrl_motor_mixer_command;
motor_command_t       ctrl_motor_command_non_restricted;
motor_command_t       ctrl_motor_command;
uint32_t              last_ctrl_update;
uint32_t              last_pid_update;
bool                  can_run_motors;
pid_state_t           pid_roll;
pid_state_t           pid_pitch;
pid_state_t           pid_yaw;


static log_block_data_control_loop_t log_block;

static void remove_bias_from_imu_reading(imu_reading_t* imu_no_bias, const imu_reading_t* imu_raw, const imu_reading_t* imu_bias);

static void constrain_rc_input(const rc_input_t* unconstrained, rc_input_t* constrained);

static void convert_rc_input_to_setpoint(const rc_input_t* state_rc_input, setpoint_t* setpoint);

static bool is_armed(const rc_input_t* rc_input_constrained);

static bool is_connected(const rc_input_t* rc_input_raw);

static void disconnect();

static void connect();

static void arm();

static void disarm();

static void notify_control_loop_started();

static void notify_control_loop_ended();

static int pid_controller_init();

static void pid_controller_update(const rates_t* measured, const rates_t* desired, pid_adjust_t* pid_adjust);

static void pid_controller_reset();


// If no packed received within this timeout, we consider ourself NOT connected.
#define CONNECTED_TIMEOUT_MS 500

static bool debug_print = false;


int controller_init() {
    last_ctrl_update = time_us_32();

    int result = pid_controller_init();

    return result;
}

void controller_update() {
    notify_control_loop_started();

    //notify_control_loop_ended();
    //return;

    uint32_t ctrl_loop_started = time_us_32();
    float ctrl_loop_dt_s = (float) (ctrl_loop_started - last_ctrl_update) / 1000000.0;

    // Step X. Read data from IMU
    imu_read(&imu_raw);

    // Step X. Remove bias from imu readings
    const imu_reading_t* imu_bias_ = imu_get_bias();
    memcpy(&imu_bias, imu_bias_, sizeof(imu_reading_t));
    remove_bias_from_imu_reading(&imu_no_bias, &imu_raw, imu_bias_);

    // Step X. Filter IMU data?
    imu_filter_gyro((vector_3d_t*) &imu_filtered.gyro_x, (vector_3d_t*) &imu_no_bias.gyro_x);

    // Step X. IMU reading to attitude rates
    ctrl_attitude_rates_measured.roll  = imu_filtered.gyro_x;
    ctrl_attitude_rates_measured.pitch = imu_filtered.gyro_y;
    ctrl_attitude_rates_measured.yaw   = imu_filtered.gyro_z;

    // Step X. Get latest data from receiver
    receiver_get_last_packet(&ctrl_rc_input_raw);

    bool connected = is_connected(&ctrl_rc_input_raw);
    if (connected != state.is_connected) {
        if (connected) {
            connect();
        } else {
            disconnect();
        }
    }

    if (state.is_connected) {
        // Constrain/crop RC input in case they're out of expected range.
        constrain_rc_input(&ctrl_rc_input_raw, &ctrl_rc_input_constrained);

        // Step X. Map receiver data to desired rotation rates.
        convert_rc_input_to_setpoint(&ctrl_rc_input_constrained, &setpoint);
    }

    // Handle states
    bool armed = is_armed(&ctrl_rc_input_constrained);
    if (armed != state.is_armed) {
        if (armed) {
            arm();
        } else {
            disarm();
        }
    }

    if (state.is_armed && state.is_connected) {
        can_run_motors = true;
    } else {
        can_run_motors = false;
    }

    if (!can_run_motors) {
        setpoint.rates.roll  = 0;
        setpoint.rates.pitch = 0;
        setpoint.rates.yaw   = 0;
        setpoint.throttle    = 0;
    }

    // Step X. Update PID values with IMU reading and desired rotation rates.
    pid_controller_update(&ctrl_attitude_rates_measured, &setpoint.rates, &ctrl_attitude_rates_adjust);

    // Step X. Pass pid values to motor mixer to get commands for motors.
    motor_mixer_update(setpoint.throttle, &ctrl_attitude_rates_adjust, &ctrl_motor_mixer_command);

    // Step X. Map motor output values to appropriate values
    // Map to values between 1000-2000, then convert to float between 0-1
    ctrl_motor_command_non_restricted.m1 = (constrain(ctrl_motor_mixer_command.m1, THROTTLE_MIN, THROTTLE_MAX) - 1000) / 1000.0;
    ctrl_motor_command_non_restricted.m2 = (constrain(ctrl_motor_mixer_command.m2, THROTTLE_MIN, THROTTLE_MAX) - 1000) / 1000.0;
    ctrl_motor_command_non_restricted.m3 = (constrain(ctrl_motor_mixer_command.m3, THROTTLE_MIN, THROTTLE_MAX) - 1000) / 1000.0;
    ctrl_motor_command_non_restricted.m4 = (constrain(ctrl_motor_mixer_command.m4, THROTTLE_MIN, THROTTLE_MAX) - 1000) / 1000.0;

    if (can_run_motors) {
        ctrl_motor_command.m1 = ctrl_motor_command_non_restricted.m1;
        ctrl_motor_command.m2 = ctrl_motor_command_non_restricted.m2;
        ctrl_motor_command.m3 = ctrl_motor_command_non_restricted.m3;
        ctrl_motor_command.m4 = ctrl_motor_command_non_restricted.m4;
    } else {
        ctrl_motor_command.m1 = 0;
        ctrl_motor_command.m2 = 0;
        ctrl_motor_command.m3 = 0;
        ctrl_motor_command.m4 = 0;
    }

    notify_control_loop_ended();
}

void controller_debug() {
   printf("bR: %.4f, x: %.4f, y: %.4f, z: %.4f\n", imu_bias.gyro_x, imu_raw.gyro_x, imu_raw.gyro_y, imu_raw.gyro_z);

   printf(
    "rX: %.3f, nBX: %.3f, gX: %.3f, setpoint: %.3f, pid: %.3f, err: %.3f, err_integral: %.3f, p: %.3f, i: %.3f, d: %.3f\n",
    imu_raw.gyro_x,
    imu_no_bias.gyro_x,
    imu_filtered.gyro_x,
    setpoint.rates.roll,
    pid_roll.pid,
    pid_roll.err,
    pid_roll.err_integral,
    pid_roll.p,
    pid_roll.i,
    pid_roll.d
   );
}

void controller_set_motors() {
    set_motor_pwm(MOTOR_DEBUG, ctrl_motor_command.m1);
    set_all_motors_pwm(&ctrl_motor_command);
}

// -- Helper functions -- //
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
    return ((time_us_32() - rc_input_raw->timestamp) < (CONNECTED_TIMEOUT_MS * 1000));
}

static bool is_armed(const rc_input_t* rc_input_constrained) {
    uint16_t arm_value = ctrl_rc_input_constrained.channels[RC_CHANNEL_ARM];
    return (arm_value >= arm_range.min) && (arm_value <= arm_range.max);
}

static void constrain_rc_input(const rc_input_t* unconstrained, rc_input_t* constrained) {
    for (int i = 0; i < RC_NBR_OF_CHANNELS ; i++) {
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
    setpoint->rates.yaw   = -RATES_ROLL_MAX  + ( ( (yaw   - 1000) / 1000.0 ) * 2*RATES_ROLL_MAX );
    setpoint->throttle = throttle;
}

int pid_controller_init() {
    last_pid_update = time_us_32();

    memset(&pid_roll, 0, sizeof(pid_state_t));
    pid_roll.Kp = 2.5;
    pid_roll.Ki = 0.25;
    pid_roll.Kd = 0;
    pid_roll.integral_limit_threshold = 100;

    memset(&pid_pitch, 0, sizeof(pid_state_t));
    pid_pitch.Kp = 2.5;
    pid_pitch.Ki = 0.25;
    pid_pitch.Kd = 0;
    pid_pitch.integral_limit_threshold = 100;

    memset(&pid_yaw, 0, sizeof(pid_state_t));
    pid_yaw.Kp = 0.25;
    pid_yaw.Ki = 0.5;
    pid_yaw.Kd = 0;
    pid_yaw.integral_limit_threshold = 100;

    return 0;
}


void pid_controller_update(const rates_t* measured, const rates_t* desired, pid_adjust_t* adjust) {
    float dt_s = (float) (time_us_32() - last_pid_update) / 1000000.0;
    adjust->roll  = pid_update(&pid_roll,  measured->roll,  desired->roll,  dt_s);
    adjust->pitch = pid_update(&pid_pitch, measured->pitch, desired->pitch, dt_s);
    adjust->yaw   = pid_update(&pid_yaw,   measured->yaw,   desired->yaw,   dt_s);
    last_pid_update = time_us_32();
}

void pid_controller_reset() {
    // TODO: Better way of this?
    pid_controller_init();
}


static void notify_control_loop_started() {

}

static void notify_control_loop_ended() {
    #ifdef TELEMETRY_LOGGING
        log_block.raw_gyro_x = imu_raw.gyro_x;
        log_block.raw_gyro_y = imu_raw.gyro_y;
        log_block.raw_gyro_z = imu_raw.gyro_z;
        log_block.filtered_gyro_x;
        log_block.filtered_gyro_y;
        log_block.filtered_gyro_z;
        log_block.rc_in_roll;
        log_block.rc_in_pitch;
        log_block.rc_in_yaw;
        log_block.rc_in_throttle;
        log_block.setpoint_roll;
        log_block.setpoint_pitch;
        log_block.setpoint_yaw;
        log_block.setpoint_throttle;
        log_block.is_connected;
        log_block.is_armed;
        log_block.can_run_motors;
        // Goes for Roll, Pitch & Yaw:
        log_block.roll_error;
        log_block.roll_error_integral;
        log_block.roll_p;
        log_block.roll_i;
        log_block.roll_d;
        log_block.roll_pid;
        log_block.roll_adjust;

        log_block.pitch_error;
        log_block.pitch_error_integral;
        log_block.pitch_p;
        log_block.pitch_i;
        log_block.pitch_d;
        log_block.pitch_pid;
        log_block.pitch_adjust;

        log_block.yaw_error;
        log_block.yaw_error_integral;
        log_block.yaw_p;
        log_block.yaw_i;
        log_block.yaw_d;
        log_block.yaw_pid;
        log_block.yaw_adjust;

        log_block.m1_non_restricted;
        log_block.m2_non_restricted;
        log_block.m3_non_restricted;
        log_block.m4_non_restricted;

        log_block.m1_restricted;
        log_block.m2_restricted;
        log_block.m3_restricted;
        log_block.m4_restricted;

        log_block.battery;

        telemetry_send_state((log_block_data_t*) &log_block, LOG_TYPE_PID);
    #endif
}
#include "pid_controller.h"
#include "pid.h"
#include "imu.h"
#include "receiver.h"
#include "motor_mixer.h"
#include "motor.h"
#include "state.h"
#include "led.h"

#include "pico/stdlib.h"


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

#define RATES_ROLL_MIN  0
#define RATES_ROLL_MAX  250
#define RATES_PITCH_MIN 0
#define RATES_PITCH_MAX 250
#define RATES_YAW_MIN   0
#define RATES_ROLL_MAX  250

#define constrain(val, min, max) ( (val < min) ? min : ( (val > max) ? max : val ) )


typedef struct {
    uint16_t throttle;
    rates_t rates;
} setpoint_t;



// Intermediate variables used to calculate correct commands for motors
//   from the RC input.
// These purpose of having variables for each step in the control loop
//   is to make debugging and logging easier.
static imu_reading_t         imu_reading;
static rates_t               ctrl_attitude_rates_measured;
static rc_input_t            ctrl_rc_input_raw;
static rc_input_t            ctrl_rc_input_constrained;
static setpoint_t            setpoint;
static pid_adjust_t          ctrl_attitude_rates_adjust; // @cal;
static motor_mixer_command_t ctrl_motor_mixer_command;
static motor_command_t       ctrl_motor_command;
static uint32_t              last_update;
static bool                  can_run_motors;

// If no packed received within this timeout, we consider ourself NOT connected.
#define CONNECTED_TIMEOUT_MS 500

static bool debug_print = false;

static void constrain_rc_input(const rc_input_t* unconstrained, rc_input_t* constrained);

static void convert_rc_input_to_setpoint(const rc_input_t* state_rc_input, setpoint_t* setpoint);

static bool is_armed(const rc_input_t* rc_input_constrained);

static bool is_connected(const rc_input_t* rc_input_raw);


int controller_init() {
    last_update = time_us_32();
    return 0;
}

void controller_update() {
    // Step X. Read data from IMU
    imu_read(&imu_reading);

    // Step X. Filter IMU data?

    // Step X. IMU reading to attitude rates
    ctrl_attitude_rates_measured.roll  = imu_reading.gyro_x;
    ctrl_attitude_rates_measured.pitch = imu_reading.gyro_y;
    ctrl_attitude_rates_measured.yaw   = imu_reading.gyro_z;

    // Step X. Get latest data from receiver
    receiver_get_last_packet(&ctrl_rc_input_raw);

    if (debug_print) {
        printf("Raw RC input: ");
        for (int i = 0; i < 14; i++) {
            printf("%d ", ctrl_rc_input_raw.channels[i]);
        }
        printf("\n");
    }

    // Constrain/crop RC input in case they're out of expected range.
    constrain_rc_input(&ctrl_rc_input_raw, &ctrl_rc_input_constrained);

    if (debug_print) {
        printf("Constrained RC input: ");
        for (int i = 0; i < 14; i++) {
            printf("%d ", ctrl_rc_input_constrained.channels[i]);
        }
        printf("\n");
    }

    // Step X. Map receiver data to desired rotation rates.
    convert_rc_input_to_setpoint(&ctrl_rc_input_constrained, &setpoint);
    if (debug_print) {
        printf("Setpoint: ");
        printf("Roll: %f, Pitch: %f, Yaw: %f, Throttle: %d\n",
            setpoint.rates.roll,
            setpoint.rates.pitch,
            setpoint.rates.yaw,
            setpoint.throttle
        );
    }

    // Step X. Update PID values with IMU reading and desired rotation rates.
    pid_controller_update(&ctrl_attitude_rates_measured, &setpoint.rates, &ctrl_attitude_rates_adjust);

    if (debug_print) {
        printf("PID adjust values: ");
        printf("Roll: %d, Pitch: %d, Yaw: %d\n",
            ctrl_attitude_rates_adjust.roll,
            ctrl_attitude_rates_adjust.pitch,
            ctrl_attitude_rates_adjust.yaw
        );
    }

    // Step X. Pass pid values to motor mixer to get commands for motors.
    motor_mixer_update(setpoint.throttle, &ctrl_attitude_rates_adjust, &ctrl_motor_mixer_command);

    if (debug_print) {
        printf("Motor mixer cmds: ");
        printf("M1: %d, M2: %d, M3: %d, M4: %d\n",
            ctrl_motor_mixer_command.m1,
            ctrl_motor_mixer_command.m2,
            ctrl_motor_mixer_command.m3,
            ctrl_motor_mixer_command.m4
        );
    }

    state.is_armed = is_armed(&ctrl_rc_input_constrained);
    state.is_connected = is_connected(&ctrl_rc_input_raw);
    led_set(LED_RED, state.is_armed);
    led_set(LED_GREEN, state.is_connected);

    if (state.is_armed && state.is_connected) {
        can_run_motors = true;
    } else {
        can_run_motors = false;
    }

    // Step X. Map motor output values to appropriate values
    ctrl_motor_command.m1 = (constrain(ctrl_motor_mixer_command.m1, 1000, 2000) - 1000) / 1000.0;
    ctrl_motor_command.m2 = (constrain(ctrl_motor_mixer_command.m2, 1000, 2000) - 1000) / 1000.0;
    ctrl_motor_command.m3 = (constrain(ctrl_motor_mixer_command.m3, 1000, 2000) - 1000) / 1000.0;
    ctrl_motor_command.m4 = (constrain(ctrl_motor_mixer_command.m4, 1000, 2000) - 1000) / 1000.0;

    if (debug_print) {
        printf("PWM Values for motors: ");
        printf("M1: %f, M2: %f, M3: %f, M4: %f\n",
            ctrl_motor_command.m1,
            ctrl_motor_command.m2,
            ctrl_motor_command.m3,
            ctrl_motor_command.m4
        );
        printf("\n");
    }

    if (!can_run_motors) {
        ctrl_motor_command.m1 = 0;
        ctrl_motor_command.m2 = 0;
        ctrl_motor_command.m3 = 0;
        ctrl_motor_command.m4 = 0;
    }

    // Step X. Set motor speeds
    set_all_motors_pwm(&ctrl_motor_command);
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

#include "controller.h"

#include "pid_controller.h"
#include "pid.h"
#include "state.h"
#include "led.h"
#include "string.h"



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
#define THROTTLE_MIN 1150

#define RATES_ROLL_MIN  0
#define RATES_ROLL_MAX  100
#define RATES_PITCH_MIN 0
#define RATES_PITCH_MAX 100
#define RATES_YAW_MIN   0
#define RATES_ROLL_MAX  100

#define constrain(val, min, max) ( (val < min) ? min : ( (val > max) ? max : val ) )



// Intermediate variables used to calculate correct commands for motors
//   from the RC input.
// These purpose of having variables for each step in the control loop
//   is to make debugging and logging easier.
imu_reading_t         imu_reading;
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
uint32_t              last_update;
bool                  can_run_motors;

static void remove_bias_from_imu_reading(const imu_reading_t* imu_reading, imu_reading_t* imu_filtered, const imu_reading_t* imu_bias);

static void constrain_rc_input(const rc_input_t* unconstrained, rc_input_t* constrained);

static void convert_rc_input_to_setpoint(const rc_input_t* state_rc_input, setpoint_t* setpoint);

static bool is_armed(const rc_input_t* rc_input_constrained);

static bool is_connected(const rc_input_t* rc_input_raw);

static void disconnect();

static void connect();

static void arm();

static void disarm();


// If no packed received within this timeout, we consider ourself NOT connected.
#define CONNECTED_TIMEOUT_MS 500

static bool debug_print = false;


int controller_init() {
    last_update = time_us_32();
    return 0;
}

void controller_update() {
    // Step X. Read data from IMU
    imu_read(&imu_reading);

    // Step X. Remove bias from imu readings
    const imu_reading_t* imu_bias_ = imu_get_bias();
    memcpy(&imu_bias, imu_bias_, sizeof(imu_reading_t));
    remove_bias_from_imu_reading(&imu_reading, &imu_no_bias, imu_bias_);

    // Step X. Filter IMU data?
    imu_filter_gyro((vector_3d_t*) &imu_no_bias.gyro_x, (vector_3d_t*) &imu_filtered.gyro_x);

    // Step X. IMU reading to attitude rates
    ctrl_attitude_rates_measured.roll  = imu_reading.gyro_x;
    ctrl_attitude_rates_measured.pitch = imu_reading.gyro_y;
    ctrl_attitude_rates_measured.yaw   = imu_reading.gyro_z;

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

    if (debug_print) {
        printf("IMU: ");
        printf("T: %d, Gx: %f, Gy: %f, Gz: %f",
            imu_reading.timestamp_us,
            imu_reading.gyro_x,
            imu_reading.gyro_y,
            imu_reading.gyro_z
        );
        printf("\n");

        printf("IMU bias: ");
        printf("Gx: %f, Gy: %f, Gz: %f",
            imu_bias.gyro_x,
            imu_bias.gyro_y,
            imu_bias.gyro_z
        );
        printf("\n");

        printf("IMU filtered: ");
        printf("Gx: %f, Gy: %f, Gz: %f",
            imu_filtered.gyro_x,
            imu_filtered.gyro_y,
            imu_filtered.gyro_z
        );
        printf("\n");

        printf("Raw RC input: ");
        for (int i = 0; i < 14; i++) {
            printf("%d ", ctrl_rc_input_raw.channels[i]);
        }
        printf("\n");

        printf("Constrained RC input: ");
        for (int i = 0; i < 14; i++) {
            printf("%d ", ctrl_rc_input_constrained.channels[i]);
        }
        printf("\n");

        printf("Setpoint: ");
        printf("Roll: %f, Pitch: %f, Yaw: %f, Throttle: %d\n",
            setpoint.rates.roll,
            setpoint.rates.pitch,
            setpoint.rates.yaw,
            setpoint.throttle
        );

        printf("PID adjust values: ");
        printf("Roll: %f, Pitch: %f, Yaw: %f\n",
            ctrl_attitude_rates_adjust.roll,
            ctrl_attitude_rates_adjust.pitch,
            ctrl_attitude_rates_adjust.yaw
        );

        printf("Motor mixer cmds: ");
        printf("M1: %f, M2: %f, M3: %f, M4: %f\n",
            ctrl_motor_mixer_command.m1,
            ctrl_motor_mixer_command.m2,
            ctrl_motor_mixer_command.m3,
            ctrl_motor_mixer_command.m4
        );

        printf("Motor mixer cmds (non-restricted): ");
        printf("M1: %f, M2: %f, M3: %f, M4: %f\n",
            ctrl_motor_command_non_restricted.m1,
            ctrl_motor_command_non_restricted.m2,
            ctrl_motor_command_non_restricted.m3,
            ctrl_motor_command_non_restricted.m4
        );

        printf("PWM Values for motors: ");
        printf("M1: %f, M2: %f, M3: %f, M4: %f\n",
            ctrl_motor_command.m1,
            ctrl_motor_command.m2,
            ctrl_motor_command.m3,
            ctrl_motor_command.m4
        );
        printf("\n");
    }
}

void controller_debug() {
    /*
    printf(
        "rawX: %f, rawY: %f, rawZ: %f, nbX: %f, nbY: %f, nbZ: %f, X: %f, Y: %f, Z: %f\n",
        imu_reading.gyro_x,
        imu_reading.gyro_y,
        imu_reading.gyro_z,
        imu_no_bias.gyro_x,
        imu_no_bias.gyro_y,
        imu_no_bias.gyro_z,
        imu_filtered.gyro_x,
        imu_filtered.gyro_y,
        imu_filtered.gyro_z
    );*/
    /*
    printf(
        "setX: %f, setY: %f, setZ: %f, mesX: %f, mesY: %f, mesZ: %f\n",
        imu_filtered.gyro_x,
        imu_filtered.gyro_y,
        imu_filtered.gyro_z,
        setpoint.rates.roll,
        setpoint.rates.pitch,
        setpoint.rates.yaw
    );
    */
   
   printf(
    "Pitch: %f, Err: %f, PidP: %f, Err: %f PidY: %f, Err: %f\n",
    ctrl_attitude_rates_adjust.roll,
    pid_roll.err_integral,
    ctrl_attitude_rates_adjust.pitch,
    pid_pitch.err_integral,
    ctrl_attitude_rates_adjust.yaw,
    pid_yaw.err_integral
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

static void remove_bias_from_imu_reading(const imu_reading_t* imu_reading, imu_reading_t* imu_filtered, const imu_reading_t* imu_bias) {
    imu_filtered->acc_x  = imu_reading->acc_x  - imu_bias->acc_x;
    imu_filtered->acc_y  = imu_reading->acc_y  - imu_bias->acc_y;
    imu_filtered->acc_z  = imu_reading->acc_z  - imu_bias->acc_z;
    imu_filtered->gyro_x = imu_reading->gyro_x - imu_bias->gyro_x;
    imu_filtered->gyro_y = imu_reading->gyro_y - imu_bias->gyro_y;
    imu_filtered->gyro_z = imu_reading->gyro_z - imu_bias->gyro_z;
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

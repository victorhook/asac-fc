#include "pid_controller.h"
#include "pid.h"
#include "imu.h"
#include "receiver.h"
#include "motor_mixer.h"
#include "motor.h"

#include "pico/stdlib.h"


#define RC_NBR_OF_CHANNELS 14

// RC Input mappings, should they be here?
#define RC_CHANNEL_THROTTLE 2
#define RC_CHANNEL_ROLL 0
#define RC_CHANNEL_PITCH 1
#define RC_CHANNEL_YAW 3
#define RC_CHANNEL_ARM 4

// If above range, we arm
#define RC_ARM_RANGE {1500, 2000}

#define RATES_ROLL_MIN  0
#define RATES_ROLL_MAX  250
#define RATES_PITCH_MIN 0
#define RATES_PITCH_MAX 250
#define RATES_YAW_MIN   0
#define RATES_ROLL_MAX  250

typedef struct {
    uint16_t throttle;
    rates_t rates;
} setpoint_t;



static imu_reading_t imu_reading;
static rates_t state_attitude_rates_measured;
static setpoint_t setpoint;
static pid_adjust_t state_attitude_rates_adjust; // @cal;
static rc_input_t state_rc_input_raw;
static rc_input_t state_rc_input_constrained;
static motor_command_t state_motor_command;
static uint64_t last_update;

static void constrain_rc_input(const rc_input_t* unconstrained, rc_input_t* constrained);

static void convert_rc_input_to_setpoint(const rc_input_t* state_rc_input, setpoint_t* setpoint);

int controller_init() {
    last_update = time_us_64();
    return 0;
}

void controller_update() {
    // Step X. Read data from IMU
    imu_read(&imu_reading);

    // Step X. Filter IMU data?

    // Step X. IMU reading to attitude rates
    state_attitude_rates_measured.roll  = imu_reading.gyro_x;
    state_attitude_rates_measured.pitch = imu_reading.gyro_y;
    state_attitude_rates_measured.yaw   = imu_reading.gyro_z;

    // Step X. Get latest data from receiver
    receiver_get_last_packet(&state_rc_input_raw);

    for (int i = 0; i < 14; i++) {
        printf("%d ", state_rc_input_raw.channels[i]);
    }
    printf("\n");

    // Constrain/crop RC input in case they're out of expected range.
    constrain_rc_input(&state_rc_input_raw, &state_rc_input_constrained);
    for (int i = 0; i < 14; i++) {
        printf("%d ", state_rc_input_constrained.channels[i]);
    }
    printf("\n");

    // Step X. Map receiver data to desired rotation rates.
    convert_rc_input_to_setpoint(&state_rc_input_constrained, &setpoint);
    printf("Roll: %f, Pitch: %f, Yaw: %f, Throttle: %d\n",
        setpoint.rates.roll,
        setpoint.rates.pitch,
        setpoint.rates.yaw,
        setpoint.throttle
    );

    // Step X. Update PID values with IMU reading and desired rotation rates.
    pid_controller_update(&state_attitude_rates_measured, &setpoint.rates, &state_attitude_rates_adjust);

    printf("R: %d, P: %d, Y: %d\n",
        state_attitude_rates_adjust.roll,
        state_attitude_rates_adjust.pitch,
        state_attitude_rates_adjust.yaw
    );

    // Step X. Pass pid values to motor mixer to get commands for motors.
    motor_mixer_update(setpoint.throttle, &state_attitude_rates_adjust, &state_motor_command);

    printf("M1: %d, M2: %d, M3: %d, M4: %d\n",
        state_motor_command.m1,
        state_motor_command.m2,
        state_motor_command.m3,
        state_motor_command.m4
    );

    printf("\n");
    return;

    // Step X. Map motor output values to appropriate values
    // TODO:

    // Step X. Set motor speeds
    //set_motor_speeds(&state_motor_command);
}

static int16_t inline constrain(int16_t real, int16_t low, int16_t max) {
    if (real < low)
        return low;
    if (real > max)
        return max;
    return real;
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

#include "motor.h"
#include "oneshot.h"
#include "util.h"

typedef enum {
    ESC_PROTOCOL_PWM,         // Pulse duration: 1000us - 2000us, Freq: 50 Hz
    ESC_PROTOCOL_ONESHOT_125, // Pulse duration: 125us - 250us,   Freq: Up to 4 kHz
    ESC_PROTOCOL_ONESHOT_42,  // Pulse duration: 42us - 84us,     Freq: Up to 11.9 kHz
    ESC_PROTOCOL_MULTISHOT,   // Pulse duration: (5us - 25us)      Freq: ? kHz
} esc_protocol_t;

#define PWM_PULSE_WIDTH          ((float) 0.05)
#define ONESHOT_125_PULSE_WIDTH  ((float) 0.125)

typedef struct {
    uint32_t     gpio;
    uint16_t wrap;
    uint32_t     slice;
    uint32_t     channel;
} pwm_t;

// TODO: Clean up logic here and make this a setting!
// TODO: Make this ifdef instead perhaps?
static esc_protocol_t esc_protocol = ESC_PROTOCOL_ONESHOT_42;
static float pulse_width = ONESHOT_125_PULSE_WIDTH;


extern float mot_pwm_max;
extern float mot_pwm_min;
extern float mot_spin_arm;
extern float mot_spin_max;


#define MOTOR_PWM_LEVEL_SCALER ((uint16_t) 10000)

static pwm_t pwm_m1;
static pwm_t pwm_m2;
static pwm_t pwm_m3;
static pwm_t pwm_m4;
static pwm_t pwm_m_debug;

motor_output_t motors;

static pwm_t* pwm_motors[] = {
    &pwm_m_debug,
    &pwm_m1,
    &pwm_m2,
    &pwm_m3,
    &pwm_m4
};

// -- Helper functions -- //

static inline void pwm_set(const pwm_t* pwm, const float duty);

static inline void pwm_set_level(const pwm_t* pwm, const uint16_t level);

static void init_pwm(pwm_t* pwm, uint32_t gpio, float clk_divider, uint16_t wrap);


// -- Public API -- //

int motors_init() {

    uint16_t wrap;
    float clk_div;

    switch (esc_protocol) {
        case ESC_PROTOCOL_PWM:
            // Clock divider is given by:
            // clk_div = sysclk / (freq * wrap)
            // PWM 50 Hz:
            wrap = MOTOR_PWM_LEVEL_SCALER;
            clk_div = 250.0;
            //init_pwm(&pwm_m1, PIN_M1, clk_div, wrap);
            //init_pwm(&pwm_m2, PIN_M2, clk_div, wrap);
            //init_pwm(&pwm_m3, PIN_M3, clk_div, wrap);
            //init_pwm(&pwm_m4, PIN_M4, clk_div, wrap);
            break;
        case ESC_PROTOCOL_ONESHOT_125:
            //oneshot_init(ONESHOT_TYPE_125);
            break;
        case ESC_PROTOCOL_ONESHOT_42:
            //oneshot_init(ONESHOT_TYPE_42);
            break;
        case ESC_PROTOCOL_MULTISHOT:
            break;
        default:
            // Should never happen
            return -1;
    }

    set_motor_pwm(MOTOR_1, 0.0);
    set_motor_pwm(MOTOR_2, 0.0);
    set_motor_pwm(MOTOR_3, 0.0);
    set_motor_pwm(MOTOR_4, 0.0);

    return 0;
}

void set_motor_pwm(const uint8_t motor, const float pwm) {
    float pulse;
    uint16_t pwm_value;

    switch (esc_protocol) {
        case ESC_PROTOCOL_PWM:
            // pwm: Value between 0-1.
            pulse = pulse_width + (pwm * pulse_width);
            pwm_value = pulse * MOTOR_PWM_LEVEL_SCALER;
            //printf("M: %d, Org: %f, pulse: %f, pwm_value: %d\n", motor, pwm, pulse, pwm_value);
            pwm_set_level(pwm_motors[motor], pwm_value);
            break;
        case ESC_PROTOCOL_ONESHOT_125:
        case ESC_PROTOCOL_ONESHOT_42:
            // Motor values are 1,2,3,4 ut oneshot expects 0,1,2,3
            //oneshot_set(motor-1, pwm);
            break;
        case ESC_PROTOCOL_MULTISHOT:
            break;
    }

}

void set_all_motors_pwm(const motor_output_t* motor_command) {
    set_motor_pwm(MOTOR_1, motor_command->m1);
    set_motor_pwm(MOTOR_2, motor_command->m2);
    set_motor_pwm(MOTOR_3, motor_command->m3);
    set_motor_pwm(MOTOR_4, motor_command->m4);

    if ((esc_protocol == ESC_PROTOCOL_ONESHOT_125) ||
        (esc_protocol == ESC_PROTOCOL_ONESHOT_42))
        {
            // Oneshot doesn't apply the values directly but writes them to
            // buffer, so we need to apply the values here.
            // This way, they are all synced.
            //oneshot_apply();
        }
}

// -- Private -- //

static inline void pwm_set(const pwm_t* pwm, const float duty)
{
    //pwm_set_chan_level(pwm->slice, pwm->channel, (duty / 100.0) * pwm->wrap);
}

static inline void pwm_set_level(const pwm_t* pwm, const uint16_t level)
{
    //pwm_set_chan_level(pwm->slice, pwm->channel, level);
}

static void init_pwm(pwm_t* pwm, uint32_t gpio, float clk_divider, uint16_t wrap)
{/*
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    pwm->slice = pwm_gpio_to_slice_num(gpio);
    pwm->channel = pwm_gpio_to_channel(gpio);
    pwm->wrap = wrap;

    pwm_set_clkdiv(pwm->slice, clk_divider);
    pwm_set_wrap(pwm->slice, pwm->wrap);
    pwm_set_chan_level(pwm->slice, pwm->channel, 0);
    pwm_set_enabled(pwm->slice, true);
    */
}

void motor_mixer_update(motor_output_t* output, const float roll, const float pitch, const float yaw, const float throttle)
{
    // This is mainly used for debugging and knowing if the mixer is saturated or not
    // motors.total = roll + pitch + yaw + throttle;
    // motors.c_total = constrain(motors.total, -1, 1);
    
    // Assuming Betaflights motor standard X:
    //  4 2
    //  3 1

    // Using Ardupilot default body frame, NED
    //  Roll positive  - Right
    //  Pitch positive - Upwards
    //  Yaw positive   - Right

    // Calculate throttle for each motor
    motors.m1 = -roll +  pitch + yaw + throttle;
    motors.m2 = -roll + -pitch - yaw + throttle;
    motors.m3 =  roll +  pitch - yaw + throttle;
    motors.m4 =  roll + -pitch + yaw + throttle;

    // Check if any motor is below 0. This would indicate that it should spin less than 0 which we can't.
    // In this case, we shift all motor outputs with this amount, to ensure we're in limits
    float min_motor = minf(minf(motors.m1, motors.m2), minf(motors.m3, motors.m4));
    if (min_motor < 0)
    {
        // Eg:        M1   M2   M3   M4
        //            0.6  0.2  0.5  -0.3
        // -> (+0.3)  0.9  0.5  0.8  0
        float shift = -min_motor;
        motors.m1 += shift;
        motors.m2 += shift;
        motors.m3 += shift;
        motors.m4 += shift;
    }

    // Check if any motor output is above 1, which means we're saturated too.
    // If this happens we scale all motor outputs evenly, so they are within 0-1.
    float max_motor = maxf(maxf(motors.m1, motors.m2), maxf(motors.m3, motors.m4));
    if (max_motor > 1)
    {
        // Eg:         M1   M2   M3   M4
        //             1.4  0.9  1.2  0.8
        // -> (*0.71)  1    0.6  0.9  0.6
        float scale = 1.0f / max_motor;
        motors.m1 *= scale;
        motors.m2 *= scale;
        motors.m3 *= scale;
        motors.m4 *= scale;
    }

    float pwm_min = mot_pwm_min + ((mot_pwm_max - mot_pwm_min) * mot_spin_arm);
    float pwm_max = mot_pwm_min + ((mot_pwm_max - mot_pwm_min) * mot_spin_max);
 
    // Map to pwm values
    output->m1 = (uint16_t) mapf(motors.m1, 0, 1, pwm_min, pwm_max);
    output->m2 = (uint16_t) mapf(motors.m2, 0, 1, pwm_min, pwm_max);
    output->m3 = (uint16_t) mapf(motors.m3, 0, 1, pwm_min, pwm_max);
    output->m4 = (uint16_t) mapf(motors.m4, 0, 1, pwm_min, pwm_max);
}


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

    // if (throttle < THROTTLE_MIN) {
    //     throttle = THROTTLE_MIN;
    // }
    // motor_command->m1 = throttle - adjust->roll + adjust->pitch - adjust->yaw;
    // motor_command->m2 = throttle - adjust->roll - adjust->pitch + adjust->yaw;
    // motor_command->m3 = throttle + adjust->roll + adjust->pitch + adjust->yaw;
    // motor_command->m4 = throttle + adjust->roll - adjust->pitch - adjust->yaw;
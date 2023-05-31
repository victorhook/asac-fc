#include "motor.h"
#include "machine.h"

#include "pico/stdlib.h"
#include "stdio.h"
#include "hardware/pwm.h"


typedef enum {
    ESC_PROTOCOL_PWM,         // (1000us - 2000us) Freq: 490 Hz
    ESC_PROTOCOL_ONESHOT_125, // (125us - 250us)   Freq: ? kHz
    ESC_PROTOCOL_ONESHOT_42,  // (42us - 84us)     Freq: ? kHz
    ESC_PROTOCOL_MULTISHOT,   // (5us - 25us)      Freq: ? kHz
} esc_protocol_t;

#define PWM_PULSE_WIDTH          ((float) 0.5)
#define ONESHOT_125_PULSE_WIDTH  ((float) 0.125)

typedef struct {
    uint     gpio;
    uint16_t wrap;
    uint     slice;
    uint     channel;
} pwm_t;

// TODO: Clean up logic here and make this a setting!
static esc_protocol_t esc_protocol = ESC_PROTOCOL_ONESHOT_125;
static float pulse_width = ONESHOT_125_PULSE_WIDTH;


#define MOTOR_PWM_LEVEL_SCALER ((uint16_t) 10000)

static pwm_t pwm_m1;
static pwm_t pwm_m2;
static pwm_t pwm_m3;
static pwm_t pwm_m4;
static pwm_t pwm_m_debug;

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

static void init_pwm(pwm_t* pwm, uint gpio, float clk_divider, uint16_t wrap);


// -- Public API -- //

int motors_init() {
    uint16_t wrap = MOTOR_PWM_LEVEL_SCALER;
    float clk_div;

    // Clock divider is given by:
    // clk_div = sysclk / (freq * wrap)

    switch (esc_protocol) {
        case ESC_PROTOCOL_PWM:
            // PWM 50 Hz:
            clk_div = 250.0;
            break;
        case ESC_PROTOCOL_ONESHOT_125:
            // PWM 1000 HZ:
            clk_div = 12.5;
            break;
        case ESC_PROTOCOL_ONESHOT_42:
            clk_div = 10.499790004199916;
            break;
        case ESC_PROTOCOL_MULTISHOT:
            clk_div = 3.125;
            break;
        default:
            // Should never happen
            return -1;
    }

    init_pwm(&pwm_m_debug, PIN_M_DEBUG, clk_div, wrap);
    init_pwm(&pwm_m1, PIN_M1, clk_div, wrap);
    init_pwm(&pwm_m2, PIN_M2, clk_div, wrap);
    init_pwm(&pwm_m3, PIN_M3, clk_div, wrap);
    init_pwm(&pwm_m4, PIN_M4, clk_div, wrap);

    set_motor_pwm(MOTOR_DEBUG, 0.0);
    set_motor_pwm(MOTOR_1, 0.0);
    set_motor_pwm(MOTOR_2, 0.0);
    set_motor_pwm(MOTOR_3, 0.0);
    set_motor_pwm(MOTOR_4, 0.0);

    return 0;
}

void set_motor_pwm(const uint8_t motor, const float pwm) {
    // pwm: Value between 0-1.
    float pulse = pulse_width + (pwm * pulse_width);
    uint16_t pwm_value = pulse * MOTOR_PWM_LEVEL_SCALER;
    //printf("M: %d, Org: %f, pulse: %f, pwm_value: %d\n", motor, pwm, pulse, pwm_value);
    pwm_set_level(pwm_motors[motor], pwm_value);
}

void set_all_motors_pwm(const motor_command_t* motor_command) {
    set_motor_pwm(MOTOR_1, motor_command->m1);
    set_motor_pwm(MOTOR_2, motor_command->m2);
    set_motor_pwm(MOTOR_3, motor_command->m3);
    set_motor_pwm(MOTOR_4, motor_command->m4);
}

// -- Private -- //

static inline void pwm_set(const pwm_t* pwm, const float duty)
{
    pwm_set_chan_level(pwm->slice, pwm->channel, (duty / 100.0) * pwm->wrap);
}

static inline void pwm_set_level(const pwm_t* pwm, const uint16_t level)
{
    pwm_set_chan_level(pwm->slice, pwm->channel, level);
}

static void init_pwm(pwm_t* pwm, uint gpio, float clk_divider, uint16_t wrap)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    pwm->slice = pwm_gpio_to_slice_num(gpio);
    pwm->channel = pwm_gpio_to_channel(gpio);
    pwm->wrap = wrap;

    pwm_set_clkdiv(pwm->slice, clk_divider);
    pwm_set_wrap(pwm->slice, pwm->wrap);
    pwm_set_chan_level(pwm->slice, pwm->channel, 0);
    pwm_set_enabled(pwm->slice, true);
}

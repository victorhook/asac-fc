#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "machine.h"


typedef struct {
    uint     gpio;
    uint16_t wrap;
    uint     slice;
    uint     channel;
} pwm_t;


static pwm_t pwm_m1;


void pwm_set(const pwm_t* pwm, const float duty)
{
    pwm_set_chan_level(pwm->slice, pwm->channel, (duty / 100.0) * pwm->wrap);
}


void init_pwm(pwm_t* pwm, uint gpio, float clk_divider, uint16_t wrap)
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

int motors_init() {
    init_pwm(&pwm_m1, PIN_M1, 38.3, 65465);
    pwm_set(&pwm_m1, 50);
    return 0;
}

void set_motor_pwm(float pwm) {
    pwm_set(&pwm_m1, pwm);
}
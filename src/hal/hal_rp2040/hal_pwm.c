#include "hal_impl.h"
#include <hardware/platform_defs.h>
#include <hardware/pwm.h>
#include <stdint.h>
#include "util.h"

typedef struct {
    int8_t id;
    uint32_t gpio;
    uint16_t wrap;
    uint16_t out;
    uint32_t slice;
    uint32_t channel;
} pwm_t;

#define MAX_PWM_CHANNELS 10

pwm_t pwms[MAX_PWM_CHANNELS] = {0};

static bool initialized = false;


static int8_t get_next_pwm_id()
{
    int8_t id = 0;
    for (int i = 0; i < MAX_PWM_CHANNELS; i++)
    {
        if (pwms[i].id == -1)
        {
            return id;
        }
        else
        {
            id++;
        }
    }
    return -1;
}


int hal_pwm_init(const int pin, int8_t* id)
{
    if (!initialized)
    {
        for (int i = 0; i < MAX_PWM_CHANNELS; i++)
        {
            pwms[i].id = -1;
        }
        initialized = true;
    }

    // TODO: Handle assertion here for ensuring pin is ok
    gpio_set_function(pin, GPIO_FUNC_PWM);
    
    int8_t new_id = get_next_pwm_id();
    if (new_id == -1)
    {   // Probably used all pwm channels already...
        return -1;
    }

    // Get the pwm struct and update its slice and channel fields
    pwm_t* pwm = &pwms[new_id];
    pwm->id = new_id;
    pwm->slice = pwm_gpio_to_slice_num(pin);
    pwm->channel = pwm_gpio_to_channel(pin);

    // 1us per clock tick
    float clk_divider = (float) SYS_CLK_HZ / 1000000.0f;
    pwm_set_clkdiv(pwm->slice, clk_divider);

    // Support 50 or 400 hz
    // 50hz  -> Period: 20ms
    // 400hz -> Period: 2.5ms
    int freq_hz = 50;
    pwm->wrap = (freq_hz == 50) ? 19999 : 2499;
    pwm_set_wrap(pwm->slice, pwm->wrap);

    // Set output low to start with
    pwm_set_chan_level(pwm->slice, pwm->channel, 0);

    pwm_set_enabled(pwm->slice, true);

    return 0;
}

int hal_pwm_set(const int8_t id, const uint16_t value)
{
    if (id >= MAX_PWM_CHANNELS) return -1;

    pwm_t* pwm = &pwms[id];
    pwm->out = map(value, 1000, 2000, 0, pwm->wrap);
    pwm_set_chan_level(pwm->slice, pwm->channel, pwm->out);

    return 0;
}

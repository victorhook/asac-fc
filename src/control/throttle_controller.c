#include "throttle_controller.h"

#include "util.h"

extern float mot_pwm_max;
extern float mot_pwm_min;

void throttle_control_init()
{

}

// For now, we'll scale this linearly between MOT_SPIN_ARM and MOT_SPIN_MAX
void throttle_control_update(float* throttle, const uint16_t rc_throttle)
{
    *throttle = map(rc_throttle, mot_pwm_min, mot_pwm_max, 0, 1);
}

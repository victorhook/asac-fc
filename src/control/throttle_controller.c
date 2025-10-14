#include "throttle_controller.h"

#include "util.h"

extern float mot_pwm_max;
extern float mot_pwm_min;

void throttle_control_init()
{

}

void throttle_control_update(float* throttle, const float desired_throttle)
{
    *throttle = desired_throttle;
}

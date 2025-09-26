#include "throttle_control.h"

#include "rc/rc.h"


void throttle_control_init()
{

}

void throttle_control_update(const uint16_t rc_throttle, float* throttle)
{
    // For now, we'll scale this linearly
    *throttle = mapf(rc_throttle, RC_CHANNEL_MIN, RC_CHANNEL_MAX, 0, 1);
}
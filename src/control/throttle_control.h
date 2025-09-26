
#ifndef THROTTLE_CONTROL__H
#define THROTTLE_CONTROL__H

#include <stdint.h>

/** Initializes the throttle controller */
void throttle_control_init();

/** Converts input RC throttle to a desired target throttle, between 0-1. */
void throttle_control_update(const uint16_t rc_throttle, float* throttle);


#endif

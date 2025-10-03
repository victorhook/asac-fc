#ifndef THROTTLE_CONTROLLER__H
#define THROTTLE_CONTROLLER__H

#include <stdint.h>


/** Initializes the throttle controller */
void throttle_control_init();

/** Converts input RC throttle to a desired target throttle, between 0-1. */
void throttle_control_update(float* throttle, const uint16_t rc_throttle);


#endif

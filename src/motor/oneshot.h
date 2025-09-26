#ifndef ONESHOT_H
#define ONESHOT_H

#include "stdint.h"


typedef enum {
    ONESHOT_TYPE_125,
    ONESHOT_TYPE_42
} oneshot_type_t;

/*
 * Initializes oneshot with given oneshot type.
 * Can be ONESHOT_TYPE_125 or ONESHOT_TYPE_42
 */
int oneshot_init(const oneshot_type_t oneshot_type);

/*
 * Writes the throttle value for the given motor.
 * Note that this doesn't actually apply the throttle, since you must call
 * oneshot_apply() for this to have any effect.
 */
void oneshot_set(const uint8_t motor, const float throttle);

/*
 * Applies the throttle values for the motors.
 * Used to synchronize all pulses so that they start at the same time.
 */
void oneshot_apply();

#endif /* ONESHOT_H */

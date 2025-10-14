
#ifndef HAL_PWM_H
#define HAL_PWM_H

#include <stdint.h>

/** Initializes pwm for the given pin. Returns 0 on success. */
int hal_pwm_init(const int pin, int8_t* id);

/** Sets the given pin to the pwm output of `value`, which should be in range 1000-2000 */
int hal_pwm_set(const int8_t id, const uint16_t value);

#endif
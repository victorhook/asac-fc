#ifndef MOTOR_H
#define MOTOR_H

#include "stdint.h"

#define MOTOR_DEBUG 0
#define MOTOR_1     1
#define MOTOR_2     2
#define MOTOR_3     3
#define MOTOR_4     4

typedef struct {
    float m1;
    float m2;
    float m3;
    float m4;
}__attribute__((packed)) motor_output_t;


/* Initializes all motors and sets output to 0. */
int motors_init();

/*
 * Sets the pwm level for the given motor.
 *
 * Parameters:
 *   - motor: Motor number, between 1-4
 *   - pwm: Value between 0-1 that corresponds to the motor throttle.
*/
void set_motor_pwm(const uint8_t motor, const float pwm);

/*
 * Sets values for all motors.
 *
 * Parameters:
 *   - pwm: Value between 0-1 that corresponds to the motor throttle.
*/
void set_all_motors_pwm(const motor_output_t* motor_command);

/*
 * Calculates output pwm value for each motor.
 * This value is scaled between mot_pwm_min and mot_pwm_max.
 */
void motor_mixer_update(motor_output_t* output, const float roll, const float pitch, const float yaw, const float throttle);


#endif /* MOTOR_H */

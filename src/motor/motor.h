#ifndef MOTOR_H
#define MOTOR_H

#include "stdint.h"

#define MOTOR_DEBUG 0
#define MOTOR_1     1
#define MOTOR_2     2
#define MOTOR_3     3
#define MOTOR_4     4

typedef struct {
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
} motor_output_t;

typedef enum {
    ESC_PROTOCOL_PWM = 1,         // Pulse duration: 1000us - 2000us, Freq: 50 Hz
    ESC_PROTOCOL_ONESHOT_125 = 2, // Pulse duration: 125us - 250us,   Freq: Up to 4 kHz
    ESC_PROTOCOL_ONESHOT_42 = 3,  // Pulse duration: 42us - 84us,     Freq: Up to 11.9 kHz
    ESC_PROTOCOL_MULTISHOT = 4,   // Pulse duration: (5us - 25us)      Freq: ? kHz
    ESC_PROTOCOL_BRUSHED = 5,     // For brushed motors -> Normal PWM, high frequencies
} esc_protocol_t;


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

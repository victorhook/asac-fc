#include "motor.h"
#include "mavlink.h"
#include "mavlink_driver.h"
#include "oneshot.h"
#include "pwm.h"
#include "util.h"
#include "hal.h"

#define PWM_PULSE_WIDTH          ((float) 0.05)
#define ONESHOT_125_PULSE_WIDTH  ((float) 0.125)
// TODO: Clean up logic here and make this a setting!
// TODO: Make this ifdef instead perhaps?
static float pulse_width = ONESHOT_125_PULSE_WIDTH;


extern float mot_pwm_max;
extern float mot_pwm_min;
extern float mot_spin_arm;
extern float mot_spin_max;
extern float mot_pwm_type;

extern float brd_mot1;
extern float brd_mot2;
extern float brd_mot3;
extern float brd_mot4;

#define MOTOR_PWM_LEVEL_SCALER ((uint16_t) 10000)

static bool initialzed_ok = false;
static int8_t motor_ids[4];

motor_output_t motors;


// -- Public API -- //

int motors_init() {

    uint16_t wrap;
    float clk_div;

    int res = 0;
    
    switch ((esc_protocol_t) mot_pwm_type) {
        case ESC_PROTOCOL_PWM:
            res |= hal_pwm_init(brd_mot1, &motor_ids[0]);
            res |= hal_pwm_init(brd_mot2, &motor_ids[1]);
            res |= hal_pwm_init(brd_mot3, &motor_ids[2]);
            res |= hal_pwm_init(brd_mot4, &motor_ids[3]);
            break;
        case ESC_PROTOCOL_ONESHOT_125:
        case ESC_PROTOCOL_ONESHOT_42:
        case ESC_PROTOCOL_MULTISHOT:
        default:
            gcs_printf(MAV_SEVERITY_WARNING, "Invalid ESC protocol: %d", mot_pwm_type);
            return -1;
    }

    if (res != 0) return res;

    res |= hal_pwm_set(motor_ids[0], 1000);
    res |= hal_pwm_set(motor_ids[1], 1000);
    res |= hal_pwm_set(motor_ids[2], 1000);
    res |= hal_pwm_set(motor_ids[3], 1000);

    if (res == 0)
    {
        initialzed_ok = true;
    }

    return res;
}

void set_motor_pwm(const uint8_t motor, const float pwm) {
    if (!initialzed_ok) return;

    float pulse;
    uint16_t pwm_value;

    switch ((esc_protocol_t) mot_pwm_type) {
        case ESC_PROTOCOL_PWM:
            // pwm: Value between 0-1.
            //printf("M: %d, Org: %f, pulse: %f, pwm_value: %d\n", motor, pwm, pulse, pwm_value);
            hal_pwm_set(motor_ids[motor], pwm);
            break;
        case ESC_PROTOCOL_ONESHOT_125:
        case ESC_PROTOCOL_ONESHOT_42:
            // Motor values are 1,2,3,4 ut oneshot expects 0,1,2,3
            //oneshot_set(motor-1, pwm);
            break;
        case ESC_PROTOCOL_MULTISHOT:
            break;
        default:
            break;
    }

}

void set_all_motors_pwm(const motor_output_t* motor_command) {
    set_motor_pwm(MOTOR_1, motor_command->m1);
    set_motor_pwm(MOTOR_2, motor_command->m2);
    set_motor_pwm(MOTOR_3, motor_command->m3);
    set_motor_pwm(MOTOR_4, motor_command->m4);

    if ((mot_pwm_type == ESC_PROTOCOL_ONESHOT_125) ||
        (mot_pwm_type == ESC_PROTOCOL_ONESHOT_42))
        {
            // Oneshot doesn't apply the values directly but writes them to
            // buffer, so we need to apply the values here.
            // This way, they are all synced.
            //oneshot_apply();
        }
}

// -- Private -- //
void motor_mixer_update(motor_output_t* output, const float roll, const float pitch, const float yaw, const float throttle)
{
    // This is mainly used for debugging and knowing if the mixer is saturated or not
    // motors.total = roll + pitch + yaw + throttle;
    // motors.c_total = constrain(motors.total, -1, 1);
    
    // Assuming Betaflights motor standard X:
    //  4 2
    //  3 1

    // Using Ardupilot default body frame, NED
    //  Roll positive  - Right
    //  Pitch positive - Upwards
    //  Yaw positive   - Right

    // Calculate throttle for each motor
    motors.m1 = -roll + pitch + yaw + throttle;
    motors.m2 = -roll - pitch - yaw + throttle;
    motors.m3 =  roll + pitch - yaw + throttle;
    motors.m4 =  roll - pitch + yaw + throttle;

    // Check if any motor is below 0. This would indicate that it should spin less than 0 which we can't.
    // In this case, we shift all motor outputs with this amount, to ensure we're in limits
    float min_motor = minf(minf(motors.m1, motors.m2), minf(motors.m3, motors.m4));
    if (min_motor < 0)
    {
        // Eg:        M1   M2   M3   M4
        //            0.6  0.2  0.5  -0.3
        // -> (+0.3)  0.9  0.5  0.8  0
        float shift = -min_motor;
        motors.m1 += shift;
        motors.m2 += shift;
        motors.m3 += shift;
        motors.m4 += shift;
    }

    // Check if any motor output is above 1, which means we're saturated too.
    // If this happens we scale all motor outputs evenly, so they are within 0-1.
    float max_motor = maxf(maxf(motors.m1, motors.m2), maxf(motors.m3, motors.m4));
    if (max_motor > 1)
    {
        // Eg:         M1   M2   M3   M4
        //             1.4  0.9  1.2  0.8
        // -> (*0.71)  1    0.6  0.9  0.6
        float scale = 1.0f / max_motor;
        motors.m1 *= scale;
        motors.m2 *= scale;
        motors.m3 *= scale;
        motors.m4 *= scale;
    }

    float pwm_min = mot_pwm_min + ((mot_pwm_max - mot_pwm_min) * mot_spin_arm);
    float pwm_max = mot_pwm_min + ((mot_pwm_max - mot_pwm_min) * mot_spin_max);
 
    // Map to pwm values
    output->m1 = (uint16_t) mapf(motors.m1, 0, 1, pwm_min, pwm_max);
    output->m2 = (uint16_t) mapf(motors.m2, 0, 1, pwm_min, pwm_max);
    output->m3 = (uint16_t) mapf(motors.m3, 0, 1, pwm_min, pwm_max);
    output->m4 = (uint16_t) mapf(motors.m4, 0, 1, pwm_min, pwm_max);
}


/*
    M4   M2
        \ /
        / \
    M3   M1

    M1 1, -1,  1, -1  <- Rear right
    M2 1, -1, -1,  1  <- Front right
    M3 1,  1,  1,  1  <- Rear left
    M4 1,  1, -1, -1  <- Front left
*/

// if (throttle < THROTTLE_MIN) {
//     throttle = THROTTLE_MIN;
// }
// motor_command->m1 = throttle - adjust->roll + adjust->pitch - adjust->yaw;
// motor_command->m2 = throttle - adjust->roll - adjust->pitch + adjust->yaw;
// motor_command->m3 = throttle + adjust->roll + adjust->pitch + adjust->yaw;
// motor_command->m4 = throttle + adjust->roll - adjust->pitch - adjust->yaw;
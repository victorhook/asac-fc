#include "motor_mixer.h"

/*
    M4     M2
       \ /
       / \
    M3     M1
*/
int8_t m1[] = {1, -1,  1, -1};  // Rear right
int8_t m2[] = {1, -1, -1,  1};  // Back right
int8_t m3[] = {1,  1,  1,  1};  // Rear left
int8_t m4[] = {1,  1, -1, -1};  // Front left


void motor_mixer_update(const uint16_t throttle, const pid_adjust_t* adjust, motor_command_t* motor_command) {
    motor_command->m1 = (throttle * m1[0]) + (adjust->roll * m1[1]) + (adjust->pitch * m1[2]) + (adjust->yaw * m1[3]);
    motor_command->m2 = (throttle * m2[0]) + (adjust->roll * m2[1]) + (adjust->pitch * m2[2]) + (adjust->yaw * m2[3]);
    motor_command->m3 = (throttle * m3[0]) + (adjust->roll * m3[1]) + (adjust->pitch * m3[2]) + (adjust->yaw * m3[3]);
    motor_command->m4 = (throttle * m4[0]) + (adjust->roll * m4[1]) + (adjust->pitch * m4[2]) + (adjust->yaw * m4[3]);
}


void set_motor_speeds(const motor_command_t* motor_command) {

}

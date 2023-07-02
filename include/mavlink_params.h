#ifndef MAVLINK_PARAMS_H
#define MAVLINK_PARAMS_H

#include "mavlink/asac/mavlink.h"


typedef struct {
    mavlink_param_value_t pid_gyro_roll_p;
    mavlink_param_value_t pid_gyro_roll_i;
    mavlink_param_value_t pid_gyro_roll_d;
    mavlink_param_value_t pid_gyro_pitch_p;
    mavlink_param_value_t pid_gyro_pitch_i;
    mavlink_param_value_t pid_gyro_pitch_d;
    mavlink_param_value_t pid_gyro_yaw_p;
    mavlink_param_value_t pid_gyro_yaw_i;
    mavlink_param_value_t pid_gyro_yaw_d;
}__attribute__((packed)) system_params_t;

extern system_params_t system_params;

extern const uint16_t NBR_OF_PARAM_VALUES;


#endif /* MAVLINK_PARAMS_H */
#include "mavlink_params.h"


system_params_t system_params = {
    .pid_gyro_roll_p = {
        .param_value = 99,
        .param_count = 0,
        .param_index = 0,
        .param_id = "pid_gyro_roll_p",
        .param_type = MAV_PARAM_TYPE_REAL32
    },
    .pid_gyro_roll_i = {
        .param_value = 99,
        .param_count = 0,
        .param_index = 0,
        .param_id = "pid_gyro_roll_i",
        .param_type = MAV_PARAM_TYPE_REAL32
    },
    .pid_gyro_roll_d = {
        .param_value = 99,
        .param_count = 0,
        .param_index = 0,
        .param_id = "pid_gyro_roll_d",
        .param_type = MAV_PARAM_TYPE_REAL32
    },
    .pid_gyro_pitch_p = {
        .param_value = 99,
        .param_count = 0,
        .param_index = 0,
        .param_id = "pid_gyro_pitch_p",
        .param_type = MAV_PARAM_TYPE_REAL32
    },
    .pid_gyro_pitch_i = {
        .param_value = 99,
        .param_count = 0,
        .param_index = 0,
        .param_id = "pid_gyro_pitch_i",
        .param_type = MAV_PARAM_TYPE_REAL32
    },
    .pid_gyro_pitch_d = {
        .param_value = 99,
        .param_count = 0,
        .param_index = 0,
        .param_id = "pid_gyro_pitch_d",
        .param_type = MAV_PARAM_TYPE_REAL32
    },
    .pid_gyro_yaw_p = {
        .param_value = 99,
        .param_count = 0,
        .param_index = 0,
        .param_id = "pid_gyro_yaw_p",
        .param_type = MAV_PARAM_TYPE_REAL32
    },
    .pid_gyro_yaw_i = {
        .param_value = 99,
        .param_count = 0,
        .param_index = 0,
        .param_id = "pid_gyro_yaw_i",
        .param_type = MAV_PARAM_TYPE_REAL32
    },
    .pid_gyro_yaw_d = {
        .param_value = 99,
        .param_count = 0,
        .param_index = 0,
        .param_id = "pid_gyro_yaw_d",
        .param_type = MAV_PARAM_TYPE_REAL32
    }
};

const uint16_t NBR_OF_PARAM_VALUES = (sizeof(system_params) / sizeof(mavlink_param_value_t));

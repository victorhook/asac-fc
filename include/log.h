#ifndef LOG_H
#define LOG_H

#include "stdint.h"
#include "stdbool.h"


typedef enum
{
    LOG_TYPE_PID,
    LOG_TYPE_BATTERY
} log_type_t;

typedef struct {
    float raw_gyro_x;
    float raw_gyro_y;
    float raw_gyro_z;
    float filtered_gyro_x;
    float filtered_gyro_y;
    float filtered_gyro_z;
    uint16_t rc_in_roll;
    uint16_t rc_in_pitch;
    uint16_t rc_in_yaw;
    uint16_t rc_in_throttle;
    float setpoint_roll;
    float setpoint_pitch;
    float setpoint_yaw;
    float setpoint_throttle;
    bool is_connected;
    bool is_armed;
    bool can_run_motors;
    // Goes for Roll, Pitch & Yaw:
    float roll_error;
    float roll_error_integral;
    float roll_p;
    float roll_i;
    float roll_d;
    float roll_pid;
    float roll_adjust;

    float pitch_error;
    float pitch_error_integral;
    float pitch_p;
    float pitch_i;
    float pitch_d;
    float pitch_pid;
    float pitch_adjust;

    float yaw_error;
    float yaw_error_integral;
    float yaw_p;
    float yaw_i;
    float yaw_d;
    float yaw_pid;
    float yaw_adjust;

    float m1_non_restricted;
    float m2_non_restricted;
    float m3_non_restricted;
    float m4_non_restricted;

    float m1_restricted;
    float m2_restricted;
    float m3_restricted;
    float m4_restricted;

    float battery;
} log_block_control_loop_t;


typedef struct {
    float voltage;
} log_block_battery_t;


typedef union {
    log_block_control_loop_t pid;
    log_block_battery_t bat;
} log_block_t;

typedef struct
{
    log_type_t type;
    uint32_t   timestamp;
    uint64_t   id;
}__attribute__((packed)) log_block_header_t;



#endif /* LOG_H */

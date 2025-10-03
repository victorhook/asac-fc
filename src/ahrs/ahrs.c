#include "ahrs.h"
#include "hal.h"
#include "util.h"
#include "imu/imu.h"
#include "math.h"
#include "state.h"

int ahrs_init()
{
    return 0;
}

void ahrs_update()
{
    return;
    static uint32_t last_update_us = 0;

    if (last_update_us == 0) last_update_us = hal_micros();
    
    uint32_t dt_us = hal_micros() - last_update_us;

    // Estimate attitude based on IMU only, using complementary filter
    // Resources:
    // https://www.youtube.com/watch?v=CHSYgLfhwUo&ab_channel=Code%26Supply
    // https://ahrs.readthedocs.io/en/latest/filters/tilt.html

    // Attitude estimation using complementary filter
    const float GYRO_PART = 0.995;
    const float ACCEL_PART = 1 - GYRO_PART;

    float ax = imu_filtered.acc_x;
    float ay = imu_filtered.acc_y;
    float az = imu_filtered.acc_z;

    float gx = imu_filtered.gyro_x;
    float gy = imu_filtered.gyro_y;
    float gz = imu_filtered.gyro_z;

    float acc_roll = atan2f(ay, az);
    float acc_pitch = atan2f(-ax, sqrtf(powf(ay, 2) + powf(az, 2)));

    state.roll = GYRO_PART  * (state.roll + (gx * dt_us)) +
                  ACCEL_PART * acc_roll;
    state.pitch = GYRO_PART  * (state.pitch + (gy * dt_us)) +
                  ACCEL_PART * acc_pitch;

    state.roll_speed = imu_filtered.gyro_x;
    state.pitch_speed = imu_filtered.gyro_y;
    state.yaw_speed = imu_filtered.gyro_z;
}

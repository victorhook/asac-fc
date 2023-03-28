#ifndef IMU_H
#define IMU_H


#include "stdint.h"
#include "stdio.h"


typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    uint64_t timestamp_us;
} imu_reading_t;


int imu_init();


void imu_get_latest_reading(imu_reading_t* reading);


void imu_read(imu_reading_t* reading);


#endif
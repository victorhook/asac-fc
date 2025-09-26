#ifndef IMU_H
#define IMU_H


#include "stdint.h"
#include "stdio.h"

typedef struct {
    float x;
    float y;
    float z;
}__attribute__((packed)) vector_3d_t;
typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    uint32_t timestamp_us;
}__attribute__((packed)) imu_reading_t;


int imu_init();


int imu_calibrate();


const imu_reading_t* imu_get_bias();


void imu_filter_gyro(vector_3d_t* filtered, const vector_3d_t* raw);

void imu_read(imu_reading_t* reading);


#endif

#ifndef IMU_MPU6500_H
#define IMU_MPU6500_H

#include "imu.h"


int imu_mpu6500_do_init(const bus_config_t config);

bool imu_mpu6500_do_read(imu_reading_t* imu_reading);


#endif
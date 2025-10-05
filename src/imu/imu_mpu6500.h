
#ifndef IMU_MPU6500_H
#define IMU_MPU6500_H

#include "imu.h"


int imu_mpu6500_do_init(imu_backend_bus_t* bus);

bool imu_mpu6500_do_read(imu_reading_t* imu_reading);


#endif
#ifndef MPU6050_H
#define MPU6050_H

#include "imu.h"


int imu_mpu6050_do_init(imu_backend_bus_t* bus);

bool imu_mpu6050_do_read(imu_reading_t* imu_reading);


#endif /* MPU6050_H */
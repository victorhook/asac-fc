#ifndef MPU6050_H
#define MPU6050_H

#include "imu.h"

int imu_mpu6050_do_init(imu_backend_bus_t* bus);

bool imu_mpu6050_do_read(imu_reading_t* imu_reading);


/*
typedef struct {
    int result;
    i2c_inst_t* i2c;
} mpu6050_t;


int mpu6050_init(mpu6050_t* mpu, i2c_inst_t* i2c_bus);


int mpu6050_read(mpu6050_t* mpu, float acc[3], float gyro[3]);
*/

#endif /* MPU6050_H */
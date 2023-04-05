#ifndef MPU6050_H
#define MPU6050_H


#include "hardware/i2c.h"


typedef struct {
    int result;
    i2c_inst_t* i2c;
} mpu6050_t;


int mpu6050_init(mpu6050_t* mpu, i2c_inst_t* i2c_bus);


int mpu6050_read(mpu6050_t* mpu, float acc[3], float gyro[3]);


#endif /* MPU6050_H */
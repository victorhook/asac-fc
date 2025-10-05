#ifndef IMU_H
#define IMU_H

#include "hal.h"

typedef enum
{
    IMU_TYPE_MPU6050 = 1,
    IMU_TYPE_BMI270  = 2,
    IMU_TYPE_SITL    = 3,
} imu_type_t;

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float temp;
    uint32_t timestamp_us;
}__attribute__((packed)) imu_reading_t;

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} imu_calibration_t;

typedef union
{
    i2c_t* i2c;
    spi_t* spi;
} imu_backend_bus_t;

int imu_init();

void imu_update(imu_reading_t* reading);

bool imu_calibrate_gyro();

bool imu_calibrate_accel();

void imu_get_calibration(imu_calibration_t* calibration);

void imu_set_calibration(const imu_calibration_t* calibration);

void imu_filter(imu_reading_t* filtered, const imu_reading_t* raw);

// Abstract
void imu_read(imu_reading_t* reading);


#endif
#ifndef BMI270_H
#define BMI270_H

#include "imu.h"

int imu_bmi270_do_init();

bool imu_bmi270_do_read(imu_reading_t* imu_reading);

#endif /* BMI270_H */

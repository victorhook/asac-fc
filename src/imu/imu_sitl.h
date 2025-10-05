
#ifndef HAL_SITL_H
#define HAL_SITL_H

#include "imu.h"

int imu_sitl_do_init(imu_backend_bus_t* bus);

bool imu_sitl_do_read(imu_reading_t* imu_reading);

#endif
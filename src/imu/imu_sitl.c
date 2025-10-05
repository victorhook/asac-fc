#include "imu_sitl.h"

int imu_sitl_do_init(imu_backend_bus_t* bus)
{
    return 0;
}

bool imu_sitl_do_read(imu_reading_t* imu_reading)
{
    return true;
}
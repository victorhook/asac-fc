#include "imu_sitl.h"

int imu_sitl_do_init(imu_backend_bus_t* bus)
{
    return 0;
}

bool imu_sitl_do_read(imu_reading_t* imu_reading)
{
    imu_reading->acc_x = 0;
    imu_reading->acc_y = 0;
    imu_reading->acc_z = 9.82;
    imu_reading->gyro_x = 0;
    imu_reading->gyro_y = 0;
    imu_reading->gyro_z = 0;
    return true;
}

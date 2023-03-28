#include "imu.h"
#include "machine.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "drivers/mpu6050.h"


// IMU uses I2C bus 0
imu_reading_t last_reading;
mpu6050_t mpu;

int imu_init() {
    // Initialize i2c bus and gpio
    i2c_init(I2C_BUS_IMU, 400 * 1000);
    gpio_set_function(PIN_SDA1, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL1, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA1);
    gpio_pull_up(PIN_SCL1);

    // Enable interrupts
    mpu6050_init(&mpu, I2C_BUS_IMU);
    //i2c_write_reg(REG_INT_ENABLE, REG_INT_ENABLE_DATA_RDY_EN);

    return 0;

    //mpu6050_reset();

    //return mpu6050_init(&mpu6050, I2C_BUS_IMU, 0);
}

void imu_read(imu_reading_t* reading) {
    mpu6050_read(&mpu, &reading->acc_x, &reading->gyro_x);
    reading->timestamp_us = time_us_64();

    //printf("ACC %f, %f, %f\n", reading->acc_x, reading->acc_y, reading->acc_z);
    //printf("GYRO %f, %f, %f\n", reading->gyro_x, reading->gyro_y, reading->gyro_z);
    return;
}


void imu_get_latest_reading(imu_reading_t* reading) {
    //last_reading.timestamp = time_us_64();
    //mpu6050_read_raw();
}

#include "imu.h"
#include "machine.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "drivers/mpu6050.h"

// IMU uses I2C bus 0
imu_reading_t last_reading;
mpu6050_inst_t mpu6050;
mpu6050_data raw_imu_data;


int imu_init() {
    // Initialize i2c bus and gpio
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(PIN_SDA0, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL0, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA0);
    gpio_pull_up(PIN_SCL0);

    mpu6050_init(&mpu6050, i2c0, 0);
}

void imu_read() {
    last_reading.timestamp = time_us_64();
    mpu6050_read_raw()
}


/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
}


static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}
#include "imu_mpu6050.h"
#include "hal.h"
#include "i2c.h"
#include "imu.h"
#include "mavlink.h"
#include "mavlink_driver.h"



typedef enum {
    MPU6050_REG_SMPRT_DIV          = 0x19,
    MPU6050_REG_CONFIG             = 0x1A,
    MPU6050_REG_GYRO_CONFIG        = 0x1B,
    MPU6050_REG_ACCEL_CONFIG       = 0x1C,
    MPU6050_REG_FIFO_EN            = 0x23,
    MPU6050_REG_INT_PIN_CFG        = 0x37,
    MPU6050_REG_INT_ENABLE         = 0x38,
    MPU6050_REG_INT_STATUS         = 0x3A,
    MPU6050_REG_ACCEL_XOUT_H       = 0x3B,
    MPU6050_REG_ACCEL_XOUT_L       = 0x3C,
    MPU6050_REG_ACCEL_YOUT_H       = 0x3D,
    MPU6050_REG_ACCEL_YOUT_L       = 0x3E,
    MPU6050_REG_ACCEL_ZOUT_H       = 0x3F,
    MPU6050_REG_ACCEL_ZOUT_L       = 0x40,
    MPU6050_REG_TEMP_OUT_H         = 0x41,
    MPU6050_REG_TEMP_OUT_L         = 0x42,
    MPU6050_REG_GYRO_XOUT_H        = 0x43,
    MPU6050_REG_GYRO_XOUT_L        = 0x44,
    MPU6050_REG_GYRO_YOUT_H        = 0x45,
    MPU6050_REG_GYRO_YOUT_L        = 0x46,
    MPU6050_REG_GYRO_ZOUT_H        = 0x47,
    MPU6050_REG_GYRO_ZOUT_L        = 0x48,
    MPU6050_REG_SIGNAL_PATH_RESET  = 0x68,
    MPU6050_REG_USER_CTRL          = 0x6A,
    MPU6050_REG_PWR_MGMT_1         = 0x6B,
    MPU6050_REG_PWR_MGMT_2         = 0x6C,
    MPU6050_REG_WHO_AM_I           = 0x75
} mpu6050_reg_t;

typedef enum
{
    MPU6050_ACCEL_RANGE_2G  = 0x00,  // ±2g
    MPU6050_ACCEL_RANGE_4G  = 0x08,  // ±4g
    MPU6050_ACCEL_RANGE_8G  = 0x10,  // ±8g
    MPU6050_ACCEL_RANGE_16G = 0x18   // ±16g
} mpu6050_accel_range_t;

typedef enum
{
    MPU6050_GYRO_RANGE_250DPS  = 0x00,  // ±250 °/s
    MPU6050_GYRO_RANGE_500DPS  = 0x08,  // ±500 °/s
    MPU6050_GYRO_RANGE_1000DPS = 0x10,  // ±1000 °/s
    MPU6050_GYRO_RANGE_2000DPS = 0x18   // ±2000 °/s
} mpu6050_gyro_range_t;

static float accel_sensitivity[] =
{
    [MPU6050_ACCEL_RANGE_2G  >> 3] = 16384.0f,
    [MPU6050_ACCEL_RANGE_4G  >> 3] = 8192.0f,
    [MPU6050_ACCEL_RANGE_8G  >> 3] = 4096.0f,
    [MPU6050_ACCEL_RANGE_16G >> 3] = 2048.0f
};

static float gyro_sensitivity[] =
{
    [MPU6050_GYRO_RANGE_250DPS  >> 3] = 131.0f,
    [MPU6050_GYRO_RANGE_500DPS  >> 3] = 65.5f,
    [MPU6050_GYRO_RANGE_1000DPS >> 3] = 32.8f,
    [MPU6050_GYRO_RANGE_2000DPS >> 3] = 16.4f
};

#define MPU_I2C_ADDR 0x68
static i2c_t* i2c;
static mpu6050_accel_range_t current_accel_range = MPU6050_ACCEL_RANGE_2G;
static mpu6050_gyro_range_t current_gyro_range   = MPU6050_GYRO_RANGE_250DPS;
static float accel_scale;
static float gyro_scale;

// -- Helper functions -- //
static inline int read(const uint8_t reg, uint8_t* data, const uint32_t len)
{
    return hal_i2c_read(i2c, MPU_I2C_ADDR, reg, data, len);
}

static inline uint8_t read_reg(const uint8_t reg)
{
    return hal_i2c_read_reg(i2c, MPU_I2C_ADDR, reg);
}

static inline int write_reg(const uint8_t reg, const uint8_t data)
{
    return hal_i2c_write_reg(i2c, MPU_I2C_ADDR, reg, data);
}

static void mpu6500_set_accel_range(mpu6050_accel_range_t range)
{
    write_reg(MPU6050_REG_ACCEL_CONFIG, range);
    current_accel_range = range;
    accel_scale = 1.0f / accel_sensitivity[range >> 3];
}

static void mpu6050_set_gyro_range(mpu6050_gyro_range_t range)
{
    write_reg(MPU6050_REG_GYRO_CONFIG, range);
    current_gyro_range = range;
    gyro_scale = 1.0f / gyro_sensitivity[range >> 3];
}

int imu_mpu6050_do_init(imu_backend_bus_t* bus)
{
    i2c = bus->i2c;

        // --- WHO_AM_I check ---
    uint8_t whoami = read_reg(MPU6050_REG_WHO_AM_I);
    if (whoami != 0x68 && whoami != 0x69)
    {
        gcs_printf(MAV_SEVERITY_ERROR, "Invalid whoami for MPU6050: %02x", whoami);
        return -1;
    }

    // --- Reset device ---
    write_reg(MPU6050_REG_PWR_MGMT_1, 0x80);
    hal_sleep_ms(100);

    // --- Wake up, use X-gyro PLL ---
    write_reg(MPU6050_REG_PWR_MGMT_1, 0x01);

    // --- Sample rate: 1 kHz ---
    write_reg(MPU6050_REG_SMPRT_DIV, 0x00);

    // --- DLPF: 44 Hz bandwidth ---
    write_reg(MPU6050_REG_CONFIG, 0x03);

    // Write GYRO_CONFIG = 0x00 (±250 dps).
    mpu6050_set_gyro_range(MPU6050_GYRO_RANGE_2000DPS);

    // Write ACCEL_CONFIG = 0x00 (±2g).
    mpu6500_set_accel_range(MPU6050_ACCEL_RANGE_16G);

    // --- Optional: reset signal paths ---
    write_reg(MPU6050_REG_SIGNAL_PATH_RESET, 0x07);
    hal_sleep_ms(100);

    gcs_printf(MAV_SEVERITY_INFO, "INIT IS OK");

    return 0;
}

void print_buf(const uint8_t* data, const uint32_t len)
{
    char buf[256];
    uint32_t offset = 0;

    for (uint32_t i = 0; i < len; i++) {
        int n = snprintf(buf + offset, sizeof(buf) - offset, "%02X ", data[i]);
        if (n <= 0 || offset + n >= sizeof(buf))
            break;  // avoid overflow
        offset += n;
    }

    // Remove trailing space and print
    if (offset > 0 && buf[offset - 1] == ' ')
        buf[offset - 1] = '\0';
    else
        buf[offset] = '\0';

    gcs_printf(MAV_SEVERITY_DEBUG, "%s", buf);
}

bool imu_mpu6050_do_read(imu_reading_t* imu_reading)
{
    uint8_t buf[14];
    read(MPU6050_REG_ACCEL_XOUT_H, buf, 14);

    // Combine high/low bytes (big-endian -> signed 16-bit)
    int16_t accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t temp_raw = (int16_t)((buf[6] << 8) | buf[7]);
    int16_t gyro_x  = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    imu_reading->acc_x = accel_x * accel_scale;
    imu_reading->acc_y = accel_y * accel_scale;
    imu_reading->acc_z = accel_z * accel_scale;

    imu_reading->gyro_x = gyro_x * gyro_scale;
    imu_reading->gyro_y = gyro_y * gyro_scale;
    imu_reading->gyro_z = gyro_z * gyro_scale;

    // Temperature formula from datasheet:
    // Temp in °C = (TEMP_OUT / 340) + 36.53
    imu_reading->temp = (temp_raw / 340.0f) + 36.53f;

    return true;
}

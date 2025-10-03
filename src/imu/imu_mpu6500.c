#include "imu_mpu6500.h"
#include "hal.h"

typedef enum
{
    // Power management
    MPU6500_PWR_MGMT_1      = 0x6B,
    MPU6500_PWR_MGMT_2      = 0x6C,

    // Sample rate / config
    MPU6500_SMPLRT_DIV      = 0x19,
    MPU6500_CONFIG          = 0x1A,
    MPU6500_GYRO_CONFIG     = 0x1B,
    MPU6500_ACCEL_CONFIG    = 0x1C,
    MPU6500_ACCEL_CONFIG2   = 0x1D,

    // Interrupts
    MPU6500_INT_PIN_CFG     = 0x37,
    MPU6500_INT_ENABLE      = 0x38,
    MPU6500_INT_STATUS      = 0x3A,

    // Sensor outputs
    MPU6500_ACCEL_XOUT_H    = 0x3B,
    MPU6500_ACCEL_XOUT_L    = 0x3C,
    MPU6500_ACCEL_YOUT_H    = 0x3D,
    MPU6500_ACCEL_YOUT_L    = 0x3E,
    MPU6500_ACCEL_ZOUT_H    = 0x3F,
    MPU6500_ACCEL_ZOUT_L    = 0x40,

    MPU6500_TEMP_OUT_H      = 0x41,
    MPU6500_TEMP_OUT_L      = 0x42,

    MPU6500_GYRO_XOUT_H     = 0x43,
    MPU6500_GYRO_XOUT_L     = 0x44,
    MPU6500_GYRO_YOUT_H     = 0x45,
    MPU6500_GYRO_YOUT_L     = 0x46,
    MPU6500_GYRO_ZOUT_H     = 0x47,
    MPU6500_GYRO_ZOUT_L     = 0x48,

    // WHO_AM_I (should return 0x70 for MPU6500)
    MPU6500_WHO_AM_I        = 0x75,
} mpu6500_reg_t;

typedef enum
{
    MPU6500_ACCEL_RANGE_2G  = 0x00,  // ±2g
    MPU6500_ACCEL_RANGE_4G  = 0x08,  // ±4g
    MPU6500_ACCEL_RANGE_8G  = 0x10,  // ±8g
    MPU6500_ACCEL_RANGE_16G = 0x18   // ±16g
} mpu6500_accel_range_t;

typedef enum
{
    MPU6500_GYRO_RANGE_250DPS  = 0x00,  // ±250 °/s
    MPU6500_GYRO_RANGE_500DPS  = 0x08,  // ±500 °/s
    MPU6500_GYRO_RANGE_1000DPS = 0x10,  // ±1000 °/s
    MPU6500_GYRO_RANGE_2000DPS = 0x18   // ±2000 °/s
} mpu6500_gyro_range_t;

static float accel_sensitivity[] =
{
    [MPU6500_ACCEL_RANGE_2G  >> 3] = 16384.0f,
    [MPU6500_ACCEL_RANGE_4G  >> 3] = 8192.0f,
    [MPU6500_ACCEL_RANGE_8G  >> 3] = 4096.0f,
    [MPU6500_ACCEL_RANGE_16G >> 3] = 2048.0f
};

static float gyro_sensitivity[] =
{
    [MPU6500_GYRO_RANGE_250DPS  >> 3] = 131.0f,
    [MPU6500_GYRO_RANGE_500DPS  >> 3] = 65.5f,
    [MPU6500_GYRO_RANGE_1000DPS >> 3] = 32.8f,
    [MPU6500_GYRO_RANGE_2000DPS >> 3] = 16.4f
};



extern float brd_imu_ss;

static mpu6500_accel_range_t current_accel_range = MPU6500_ACCEL_RANGE_2G;
static mpu6500_gyro_range_t current_gyro_range   = MPU6500_GYRO_RANGE_250DPS;
static float acc_scale;
static float gyro_scale;
static uint8_t ss = 0;
spi_t* spi;


static inline void cs_low()
{
    hal_gpio_set(ss, 0);
}

static inline void cs_high()
{
    hal_gpio_set(ss, 1);
}

static void write_reg(const uint8_t addr, const uint8_t value)
{
    cs_low();
    hal_spi_write_byte(spi, addr, value);
    cs_high();
}

static uint8_t read_reg(const uint8_t addr)
{
    uint8_t data;
    cs_low();
    hal_spi_read_byte(spi, addr, &data);
    cs_high();
    return data;
}

static inline void read_regs(const uint8_t addr, uint8_t* buf, uint32_t len)
{
    cs_low();
    hal_spi_read(spi, addr, buf, len);
    cs_high();
}

static void mpu6500_set_accel_range(mpu6500_accel_range_t range)
{
    write_reg(MPU6500_ACCEL_CONFIG, range);
    current_accel_range = range;
    acc_scale = accel_sensitivity[range >> 3];
}

static void mpu6500_set_gyro_range(mpu6500_gyro_range_t range)
{
    write_reg(MPU6500_GYRO_CONFIG, range);
    current_gyro_range = range;
    gyro_scale = gyro_sensitivity[range >> 3];
}



int imu_mpu6500_do_init(const bus_config_t config)
{
    if (config.bus != BUS_TYPE_SPI)
    {
        return -1;
    }

    ss = (uint8_t) brd_imu_ss;
    spi = (spi_t*) &config.spi;

        
    // Sanity check WHO_AM_I, expect 0x70.
    uint8_t whoami = read_reg(MPU6500_WHO_AM_I);
    if (whoami != 0x70)
    {
        return -1;
    }

    // Assert CS low, write PWR_MGMT_1 = 0x80, delay 100 ms.
    write_reg(MPU6500_PWR_MGMT_1, 0x80);
    hal_sleep_ms(100);

    // Write PWR_MGMT_1 = 0x01 (use gyro X as clock).
    write_reg(MPU6500_PWR_MGMT_1, 0x01);

    // Write SMPLRT_DIV = 0x00 (1 kHz).
    write_reg(MPU6500_SMPLRT_DIV, 0x00);

    // Write CONFIG = 0x03.
    write_reg(MPU6500_CONFIG, 0x03);

    // Write GYRO_CONFIG = 0x00 (±250 dps).
    mpu6500_set_gyro_range(MPU6500_GYRO_RANGE_2000DPS);

    // Write ACCEL_CONFIG = 0x00 (±2g).
    mpu6500_set_accel_range(MPU6500_ACCEL_RANGE_16G);

    // Write ACCEL_CONFIG2 = 0x03 (44 Hz).
    write_reg(MPU6500_ACCEL_CONFIG2, 0x03);

    // Scale based on current ranges
    acc_scale = accel_sensitivity[current_accel_range >> 3];
    gyro_scale = gyro_sensitivity[current_gyro_range >> 3];

    return 0;
}

bool imu_mpu6500_do_read(imu_reading_t* imu_reading)
{
    uint8_t buf[14];
    read_regs(MPU6500_ACCEL_XOUT_H, buf, 14);


    // Unpack big-endian 16-bit values
    int16_t raw_ax = (buf[0] << 8) | buf[1];
    int16_t raw_ay = (buf[2] << 8) | buf[3];
    int16_t raw_az = (buf[4] << 8) | buf[5];
    int16_t raw_temp = (buf[6] << 8) | buf[7];
    int16_t raw_gx = (buf[8] << 8) | buf[9];
    int16_t raw_gy = (buf[10] << 8) | buf[11];
    int16_t raw_gz = (buf[12] << 8) | buf[13];

    // Convert to physical units
    imu_reading->acc_x = raw_ax / acc_scale;
    imu_reading->acc_y = raw_ay / acc_scale;
    imu_reading->acc_z = raw_az / acc_scale;

    imu_reading->gyro_x = raw_gx / gyro_scale;
    imu_reading->gyro_y = raw_gy / gyro_scale;
    imu_reading->gyro_z = raw_gz / gyro_scale;

    imu_reading->temp = (raw_temp / 333.87f) + 21.0f;

    return true;
}

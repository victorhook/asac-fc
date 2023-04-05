#include "drivers/mpu6050.h"

#include "stdio.h"


#define MPU_I2C_ADDR 0x68

// -- Registers -- /
#define REG_SMPRT_DIV                     0x19  // SMPLRT_DIV 8-bit unsigned value. The Sample Rate is determined by dividing the gyroscope output rate by this value.
#define REG_CONFIG                        0x1A

#define REG_GYRO_CONFIG                   0x1B
#define REG_GYRO_CONFIG_FS_SEL_250        0b00
#define REG_GYRO_CONFIG_FS_SEL_500        0b01
#define REG_GYRO_CONFIG_FS_SEL_1000       0b10
#define REG_GYRO_CONFIG_FS_SEL_2000       0b11

#define REG_ACCEL_CONFIG                  0x1C
#define REG_ACCEL_CONFIG_AFS_SEL_2g       0b00
#define REG_ACCEL_CONFIG_AFS_SEL_4g       0b01
#define REG_ACCEL_CONFIG_AFS_SEL_8g       0b10
#define REG_ACCEL_CONFIG_AFS_SEL_16g      0b11

#define REG_ACCEL_START                   0x3B
#define REG_TEMP_START                    0x41
#define REG_GYRO_START                    0x43

#define REG_WHO_AM_I                      0x75

#define REG_SIGNAL_PATH_RESET             0x68
#define REG_SIGNAL_PATH_RESET_GYRO_RESET  0b100
#define REG_SIGNAL_PATH_RESET_ACCEL_RESET 0b010
#define REG_SIGNAL_PATH_RESET_TEMP_RESET  0b001

#define REG_PWR_MGMT_1                    0x6B
#define REG_PWR_MGMT_1_DEVICE_RESET       0b10000000
#define REG_PWR_MGMT_1_SLEEP              0b01000000

#define REG_INT_ENABLE                    0x38
#define REG_INT_ENABLE_DATA_RDY_EN        0b1

// 0 ±2g 16384 LSB/g
// 1 ±4g 8192 LSB/g
// 2 ±8g 4096 LSB/g
// 3 ±16g 2048 LSB/g
#define ACCEL_SENSITIVITY_SCALE ((float) 4096.0)

// 0 ± 250 °/s 131 LSB/°/s
// 1 ± 500 °/s 65.5 LSB/°/s
// 2 ± 1000 °/s 32.8 LSB/°/s
// 3 ± 2000 °/s 16.4 LSB/°/s
#define GYRO_SENSITIVITY_SCALE  ((float) 32.8)


// -- Helper functions -- //
static int mpu6050_read_bytes(mpu6050_t* mpu6050, const uint8_t address, uint8_t* buf, const uint8_t len);

static int mpu6050_read_reg(mpu6050_t* mpu6050, const uint8_t address, uint8_t* reg);

static int mpu6050_write_reg(mpu6050_t* mpu6050, const uint8_t reg, const uint8_t value);


int mpu6050_init(mpu6050_t* mpu, i2c_inst_t* i2c_bus) {
    mpu->i2c = i2c_bus;

    // Reset IMU. First perform complete device reset wand wait 100 ms
    mpu->result = mpu6050_write_reg(mpu, REG_PWR_MGMT_1, REG_PWR_MGMT_1_DEVICE_RESET);
    if (mpu->result < 0) {
        return mpu->result;
    }

    sleep_ms(100);
    // Then we'll reset all signal paths
    mpu6050_write_reg(mpu, REG_SIGNAL_PATH_RESET, REG_SIGNAL_PATH_RESET_GYRO_RESET | REG_SIGNAL_PATH_RESET_ACCEL_RESET | REG_SIGNAL_PATH_RESET_TEMP_RESET);
    sleep_ms(100);

    // Set power ON
    mpu6050_write_reg(mpu, REG_PWR_MGMT_1, 0);

    // Set sample rate: Sample Rate = 8000 / (1 + REG_SMPRT_DIV)
    mpu6050_write_reg(mpu, REG_SMPRT_DIV, 8);

    // Set Accel scale, Bits[4:3]
    mpu6050_write_reg(mpu, REG_ACCEL_CONFIG, REG_ACCEL_CONFIG_AFS_SEL_8g << 3);

    // Set Gyro scale, Bits[4:3]
    mpu6050_write_reg(mpu, REG_GYRO_CONFIG, REG_GYRO_CONFIG_FS_SEL_1000 << 3);

    return 0;
}



int mpu6050_read(mpu6050_t* mpu, float acc[3], float gyro[3]) {
    uint8_t acc_raw[6];
    mpu6050_read_bytes(mpu, REG_ACCEL_START, (uint8_t*) acc_raw, 6);

    uint8_t gyro_raw[6];
    mpu6050_read_bytes(mpu, REG_GYRO_START, (uint8_t*) gyro_raw, 6);

    // 1. Acceleration and gyro data is read as three int16, one for each axis x, y z.
    //    These int16 values are represented as big-endian so we need to shift first byte
    //    to the left.
    // 2. Once the bytes are shifted, we'll divide by a constant (sensitivity) value
    //    to get the correct units, eg m/s^2 for accel or deg/s for gyro.
    acc[0] = ( (int16_t) ((acc_raw[0] << 8) | acc_raw[1]) ) / ((float) ACCEL_SENSITIVITY_SCALE);
    acc[1] = ( (int16_t) ((acc_raw[2] << 8) | acc_raw[3]) ) / ((float) ACCEL_SENSITIVITY_SCALE);
    acc[2] = ( (int16_t) ((acc_raw[4] << 8) | acc_raw[5]) ) / ((float) ACCEL_SENSITIVITY_SCALE);

    gyro[0] = ( (int16_t) ((gyro_raw[0] << 8) | gyro_raw[1]) ) / ((float) GYRO_SENSITIVITY_SCALE);
    gyro[1] = ( (int16_t) ((gyro_raw[2] << 8) | gyro_raw[3]) ) / ((float) GYRO_SENSITIVITY_SCALE);
    gyro[2] = ( (int16_t) ((gyro_raw[4] << 8) | gyro_raw[5]) ) / ((float) GYRO_SENSITIVITY_SCALE);

    return 0;
}


static int mpu6050_read_bytes(mpu6050_t* mpu, const uint8_t address, uint8_t* buf, const uint8_t len) {
    int res = i2c_write_timeout_us(mpu->i2c, 0x68, &address, 1, true, 10000);
    if (res < 0) {
        return res;
    }
    return i2c_read_timeout_us(mpu->i2c, 0x68, buf, len, false, 10000);
}

static int mpu6050_read_reg(mpu6050_t* mpu, const uint8_t address, uint8_t* reg) {
    int res = i2c_write_timeout_us(mpu->i2c, 0x68, &address, 1, true, 10000);
    if (res < 0) {
        return res;
    }
    return i2c_read_timeout_us(mpu->i2c, 0x68, reg, 1, false, 10000);
}

static int mpu6050_write_reg(mpu6050_t* mpu, const uint8_t reg, const uint8_t value) {
    uint8_t buf[] = {reg, value};
    return i2c_write_timeout_us(mpu->i2c, 0x68, buf, 2, false, 10000);
}


static void printRegs() {
    uint8_t regs[] = {REG_SMPRT_DIV, REG_CONFIG, REG_GYRO_CONFIG, REG_ACCEL_CONFIG, REG_PWR_MGMT_1, REG_WHO_AM_I};
    for (int i = 0; i < sizeof(regs); i++) {
        uint8_t addr = regs[i];
        uint8_t reg;
        //mpu6050_read_reg(regs[i], &reg);
        printf("Reg %02x: 0x%02x\n", addr, reg);
    }
    printf("--------------\n");
}

static void debugPrintHex(const uint8_t* buf, const uint8_t len) {
    for (int i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
    }
    printf("\n");
}

#include "imu.h"
#include "machine.h"
#include "asac_fc.h"
#include "drivers/bmi270_asac.h"
#include "drivers/mpu6050.h"


#include <hardware/i2c.h>
#include <hardware/spi.h>


#define GYRO_FILTER_ORDER 2
static float gyro_filter_params[GYRO_FILTER_ORDER] = {0.9, 0.1};
static vector_3d_t gyro_filter_mem[GYRO_FILTER_ORDER];
static int gyro_filter_index;

//#define CALIBRATE_ON_INIT

#define CALIBRATION_SAMPLES                  1000
#define CALIBRATION_DELAY_BETWEEN_SAMPLES_MS 1
imu_reading_t last_reading;
mpu6050_t mpu;
bmi270_t bmi;

static imu_reading_t imu_bias;


int imu_init() {
    imu_bias.gyro_x = 0;
    imu_bias.gyro_y = 0;
    imu_bias.gyro_z = 0;
    imu_bias.acc_x = 0;
    imu_bias.acc_y = 0;
    imu_bias.acc_z = 0;

    int result;

    #if defined(IMU_MPU_6050_I2C)
        // Initialize i2c bus and gpio
        i2c_init(I2C_BUS_IMU, 400 * 1000);
        gpio_set_function(PIN_SDA1, GPIO_FUNC_I2C);
        gpio_set_function(PIN_SCL1, GPIO_FUNC_I2C);
        gpio_pull_up(PIN_SDA1);
        gpio_pull_up(PIN_SCL1);

        // Enable interrupts
        result = mpu6050_init(&mpu, I2C_BUS_IMU);
        if (result != 0) {
            return result;
        }
    #elif defined(IMU_BMI270_SPI)
        // Initialize SPI bus, BMI270 operates at max 10 MHz, allows
        // CPOL=0 & CPHA=0, or CPOL=1 & CPHA=1
        // The CS pin we control manually, thus setting to normal GPIO,
        // active low
        spi_init(IMU_SPI_BUS, 10000000); // 10 MHz
        spi_set_format(IMU_SPI_BUS, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
        gpio_set_function(PIN_IMU_MOSI, GPIO_FUNC_SPI);
        gpio_set_function(PIN_IMU_MISO, GPIO_FUNC_SPI);
        gpio_set_function(PIN_IMU_SCK, GPIO_FUNC_SPI);
        gpio_init(PIN_IMU_CS);
        gpio_set_dir(PIN_IMU_CS, GPIO_OUT);
        gpio_put(PIN_IMU_CS, HIGH);

        result = bmi270_asac_init(&bmi, IMU_SPI_BUS, PIN_IMU_CS);
        if (result != 0) {
            return result;
        }
    #endif

    #ifdef CALIBRATE_ON_INIT
        result = imu_calibrate();
        if (result != 0) {
            return result;
        }

        printf("IMU Calibration done, samples: %d, bias: \n", CALIBRATION_SAMPLES);
        printf("  Gx: %f, Gy: %f, Gz: %f, Ax: %f, Ay: %f, Az: %f\n",
            imu_bias.gyro_x,
            imu_bias.gyro_y,
            imu_bias.gyro_z,
            imu_bias.acc_x,
            imu_bias.acc_y,
            imu_bias.acc_z
        );
    #else
        // Pre-calibrated gyro bias. TODO: Place this in flash.
        // BMI270
        imu_bias.gyro_x = -0.050177;
        imu_bias.gyro_y = 0.225903;
        imu_bias.gyro_z = -0.710046;

        // MPU-6050
        //imu_bias.gyro_x = -2.688686;
        //imu_bias.gyro_y = -1.922470;
        //imu_bias.gyro_z = 1.760995;
        //imu_bias.acc_x  = -0.033133;
        //imu_bias.acc_y  = -0.005135;
        //imu_bias.acc_z  = 1.018919;
    #endif

    memset(gyro_filter_mem, 0, sizeof(gyro_filter_mem) / sizeof(vector_3d_t));
    gyro_filter_index = 0;

    return 0;
}

int imu_calibrate() {
    imu_reading_t calibration;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        imu_read(&calibration);
        imu_bias.gyro_x  += calibration.gyro_x;
        imu_bias.gyro_y  += calibration.gyro_y;
        imu_bias.gyro_z  += calibration.gyro_z;
        imu_bias.acc_x   += calibration.acc_x;
        imu_bias.acc_y   += calibration.acc_y;
        imu_bias.acc_z   += calibration.acc_z;
        //printf("%d Gx: %f, Gy: %f, Gz: %f",
        //    samples,
        //    calibration.gyro_x,
        //    calibration.gyro_y,
        //    calibration.gyro_z
        //);
        //printf("\n");
        sleep_ms(CALIBRATION_DELAY_BETWEEN_SAMPLES_MS);
    }

    imu_bias.gyro_x /= CALIBRATION_SAMPLES;
    imu_bias.gyro_y /= CALIBRATION_SAMPLES;
    imu_bias.gyro_z /= CALIBRATION_SAMPLES;
    imu_bias.acc_x  /= CALIBRATION_SAMPLES;
    imu_bias.acc_y  /= CALIBRATION_SAMPLES;
    imu_bias.acc_z  /= CALIBRATION_SAMPLES;

    return 0;
}

const imu_reading_t* imu_get_bias() {
    return &imu_bias;
}


void imu_read(imu_reading_t* reading) {
    #if defined(IMU_MPU_6050_I2C)
        mpu6050_read(&mpu, &reading->acc_x, &reading->gyro_x);
    #elif defined(IMU_BMI270_SPI)
        bmi270_asac_read(&bmi, &reading->acc_x, &reading->gyro_x);
    #endif

    reading->timestamp_us = time_us_32();

    //printf("ACC %f, %f, %f\n", reading->acc_x, reading->acc_y, reading->acc_z);
    //printf("GYRO %f, %f, %f\n", reading->gyro_x, reading->gyro_y, reading->gyro_z);
}


void imu_filter_gyro(vector_3d_t* filtered, const vector_3d_t* raw) {
    // Shift all samples to the left
    for (int i = GYRO_FILTER_ORDER-1; i > 0; i--) {
        memcpy(&gyro_filter_mem[i], &gyro_filter_mem[i-1], sizeof(vector_3d_t));
    }
    // Add new sample
    memcpy(&gyro_filter_mem[0], raw, sizeof(vector_3d_t));

    if (gyro_filter_index < GYRO_FILTER_ORDER) {
        gyro_filter_index++;
        return;
    }

    // Sum upp all readings
    memset(filtered, 0, sizeof(vector_3d_t));
    for (int i = 0; i < GYRO_FILTER_ORDER; i++) {
        filtered->x += gyro_filter_params[i] * gyro_filter_mem[i].x;
        filtered->y += gyro_filter_params[i] * gyro_filter_mem[i].y;
        filtered->z += gyro_filter_params[i] * gyro_filter_mem[i].z;
    }
}

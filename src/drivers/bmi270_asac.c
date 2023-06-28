#include "drivers/bmi270_asac.h"
#include "asac_fc.h"

#include "bmi270.h"
#include "bmi2_ois.h"


static struct bmi2_dev dev;
static struct bmi2_sens_data sensor_data;

#define GRAVITY_EARTH  (9.80665f)

#define GYRO_RANGE_VALUE 2000
#define GYRO_RANGE       BMI2_GYR_RANGE_2000



// -- API for used by Bosch C driver -- //

/* Reads len bytes from register "reg_addr". */
int8_t bmi270_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/* Writes len byes to register "reg_addr". */
int8_t bmi270_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/* Delays "period" microseconds. */
void bmi270_delay(uint32_t period, void *intf_ptr);

/*!
 *  @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Accel values in meter per second squared.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *
 *  @return Degree per second.
 */
static inline float lsb_to_dps(int16_t val);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Degree per second.
 */
static inline float lsb_to_dps_example(int16_t val, float dps, uint8_t bit_width);

int bmi270_asac_init(bmi270_t* bmi, spi_inst_t* spi, uint8_t cs_pin) {
    bmi->spi = spi;
    bmi->cs_pin = cs_pin;
    gpio_put(bmi->cs_pin, HIGH);

    dev.intf_ptr   = bmi;
    dev.read       = bmi270_spi_read;
    dev.write      = bmi270_write;
    dev.delay_us   = bmi270_delay;
    dev.intf       = BMI2_SPI_INTF;
    dev.aps_status = BMI2_DISABLE;


    int8_t res;

    // Initialize the sensor
    res = bmi270_init(&dev);
    if (res != 0) {
        return res;
    }

    res = bmi2_set_adv_power_save(false, &dev);
    if (res != 0) {
        return res;
    }

    // Enable gyro and accelerometer
    res = bmi2_sensor_enable((uint8_t [2]) {BMI2_ACCEL, BMI2_GYRO}, 2, &dev);
    if (res != 0) {
        return res;
    }

    // Set sensor configurations
    // TODO: Make parts of these saveable settings?
    struct bmi2_sens_config config[2];
    config[BMI2_GYRO].type = BMI2_GYRO;
    config[BMI2_GYRO].cfg.gyr.odr          = BMI2_GYR_ODR_3200HZ, // Output data rate in Hz
    config[BMI2_GYRO].cfg.gyr.bwp          = BMI2_GYR_NORMAL_MODE, // /*! Bandwidth parameter
    config[BMI2_GYRO].cfg.gyr.filter_perf  = BMI2_PERF_OPT_MODE, // /*! Filter performance mode
    config[BMI2_GYRO].cfg.gyr.range        = GYRO_RANGE, // /*! Gyroscope Range
    config[BMI2_GYRO].cfg.gyr.noise_perf   = BMI2_PERF_OPT_MODE, // /*! Selects noise performance

    config[BMI2_ACCEL].type                = BMI2_ACCEL;
    config[BMI2_ACCEL].cfg.acc.odr         = BMI2_ACC_ODR_1600HZ, // Output data rate in Hz
    config[BMI2_ACCEL].cfg.acc.bwp         = BMI2_ACC_NORMAL_AVG4, // Bandwidth parameter
    config[BMI2_ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE, // Filter performance mode
    config[BMI2_ACCEL].cfg.acc.range       = BMI2_ACC_RANGE_8G, // g-range

    res = bmi270_set_sensor_config(&config, 2, &dev);
    if (res != 0) {
        return res;
    }

    return res;
}

int bmi270_asac_read(bmi270_t* bmi, float acc[3], float gyro[3]) {
    // The bmi2 driver to read sensor data and the example function that
    // converts the data to dps takes 450 ~us
    // The implementation below takes 200 ~us instead which is quite a lot.
    // Not entirely sure why BOSCHs driver & API takes so long but let's keep
    // this for now.
    uint8_t tmp_sensor_data[BMI2_MAX_LEN];
    bmi270_spi_read(BMI2_STATUS_ADDR | BMI2_SPI_RD_MASK, tmp_sensor_data, BMI2_ACC_GYR_AUX_SENSORTIME_NUM_BYTES+1, bmi);
    // First byte is dummy byte :)
    bmi2_parse_sensor_data(&tmp_sensor_data[1], &sensor_data, &dev);
    //bmi2_get_sensor_data(&sensor_data, &dev);
    gyro[0] = lsb_to_dps(sensor_data.gyr.x);
    gyro[1] = lsb_to_dps(sensor_data.gyr.y);
    gyro[2] = lsb_to_dps(sensor_data.gyr.z);

    // Read data and convert to correct units using BOSCHs driver & examples:
    // bmi2_get_sensor_data(&sensor_data, &dev);
    // gyro[0] = lsb_to_dps_example(sensor_data.gyr.x, 2000, dev.resolution);
    // gyro[1] = lsb_to_dps_example(sensor_data.gyr.y, 2000, dev.resolution);
    // gyro[2] = lsb_to_dps_example(sensor_data.gyr.z, 2000, dev.resolution);
}


// -- Helper functions -- //

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static inline float lsb_to_dps(int16_t val)
{
    /*
    Original example:
        double power = 2;
        float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
        return (dps / (half_scale)) * (val);
    */
    #define half_scaled (float) ( ((double)pow(2.0, (double)16.0) / 2.0f) )
    #define CONVERSION  (float) ((float) GYRO_RANGE_VALUE / half_scaled)

    return CONVERSION * val;
}


static inline float lsb_to_dps_example(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

int8_t bmi270_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    gpio_put(((bmi270_t*) intf_ptr)->cs_pin, 0);
    spi_write_blocking(((bmi270_t*) intf_ptr)->spi, &reg_addr, 1);
    spi_read_blocking(((bmi270_t*) intf_ptr)->spi, 0, reg_data, len);
    gpio_put(((bmi270_t*) intf_ptr)->cs_pin, 1);
    return 0;
}

int8_t bmi270_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    gpio_put(((bmi270_t*) intf_ptr)->cs_pin, 0);
    spi_write_blocking(((bmi270_t*) intf_ptr)->spi, &reg_addr, 1);
    spi_write_blocking(((bmi270_t*) intf_ptr)->spi, reg_data, len);
    gpio_put(((bmi270_t*) intf_ptr)->cs_pin, 1);
    return 0;
}

void bmi270_delay(uint32_t period, void *intf_ptr) {
    sleep_us(period);
}

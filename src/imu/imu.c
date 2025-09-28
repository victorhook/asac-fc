#include "imu/imu.h"
#include "hal.h"
#include "mavlink.h"
#include "mavlink_driver/mavlink_driver.h"
#include <string.h>

#include "imu_bmi270.h"
#include "imu_mpu6050.h"
#include "imu_sitl.h"


//#define GYRO_FILTER_ORDER 2
//static float gyro_filter_params[GYRO_FILTER_ORDER] = {0.9, 0.1};
//static vector_3d_t gyro_filter_mem[GYRO_FILTER_ORDER];
//static int gyro_filter_index;

//#define CALIBRATE_ON_INIT

#define CALIBRATION_SAMPLES                  1000
#define CALIBRATION_DELAY_BETWEEN_SAMPLES_MS 1

static void print_calib();

// Backend handler
typedef int (*imu_do_init)();
typedef bool (*imu_do_read)(imu_reading_t*);
static int dummy_init() { return 0; }
static bool dummy_read(imu_reading_t* rc_input) { return true; }
typedef struct
{
    imu_do_init init;
    imu_do_read read;
} backend_t;


// Params
extern float imu_calib_gyro_on_boot;
extern float brd_imu_type;
extern float imu_bus;
extern float imu_offset_x;
extern float imu_offset_y;
extern float imu_offset_z;
extern float imu_accelcal_x;
extern float imu_accelcal_x;
extern float imu_accelcal_x;

static imu_calibration_t imu_calib = { 0 };

static backend_t backend;

int imu_init()
{
    switch ((imu_type_t) brd_imu_type)
    {
        case IMU_TYPE_BMI270:
            backend.init = imu_bmi270_do_init;
            backend.read = imu_bmi270_do_read;
            break;
        case IMU_TYPE_MPU6050:
            backend.init = imu_mpu6050_do_init;
            backend.read = imu_mpu6050_do_read;
            break;
        case IMU_TYPE_SITL:
            backend.init = imu_sitl_do_init;
            backend.read = imu_sitl_do_read;
            break;
        default:
            gcs_printf(MAV_SEVERITY_ERROR, "Invalid IMU type %d", (imu_type_t) brd_imu_type);
            backend.init = dummy_init;
            backend.read = dummy_read;
            break;
    }

    int result = backend.init();
    if (result != 0)
    {
        gcs_printf(MAV_SEVERITY_ERROR, "IMU init failed: %d", result);
        return result;
    }

    if (imu_calib_gyro_on_boot)
    {
        gcs_printf(MAV_SEVERITY_DEBUG, "Calibrating gyro");
        imu_calibrate_gyro();
    }
    else
    {

    }

    // TODO: Load calibration from eeprom

    return 0;
}


int imu_calibrate(const bool gyro, const bool accel) {
    imu_reading_t tmp;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        imu_read(&tmp);
        if (gyro)
        {
            imu_calib.gyro_x  += tmp.gyro_x;
            imu_calib.gyro_y  += tmp.gyro_y;
            imu_calib.gyro_z  += tmp.gyro_z;
        }
        if (accel)
        {
            imu_calib.acc_x   += tmp.acc_x;
            imu_calib.acc_y   += tmp.acc_y;
            imu_calib.acc_z   += tmp.acc_z;
        }
        hal_sleep_ms(CALIBRATION_DELAY_BETWEEN_SAMPLES_MS);
    }

    if (gyro)
    {
        imu_calib.gyro_x /= CALIBRATION_SAMPLES;
        imu_calib.gyro_y /= CALIBRATION_SAMPLES;
        imu_calib.gyro_z /= CALIBRATION_SAMPLES;
    }
    if (accel)
    {
        imu_calib.acc_x  /= CALIBRATION_SAMPLES;
        imu_calib.acc_y  /= CALIBRATION_SAMPLES;
        imu_calib.acc_z  /= CALIBRATION_SAMPLES;
    }
    return 0;
}

bool imu_calibrate_gyro()
{
    return imu_calibrate(true, false) == 0;
}

bool imu_calibrate_accel()
{
    return imu_calibrate(false, true) == 0;
}

void imu_get_calibration(imu_calibration_t* calibration)
{
    memcpy(calibration, &imu_calib, sizeof(imu_calibration_t));
}

void imu_set_calibration(const imu_calibration_t* calibration)
{
    memcpy(&imu_calib, calibration, sizeof(imu_calibration_t));
}


void imu_read(imu_reading_t* reading) {
    // 1. Read IMU data
    backend.read(reading);
    reading->timestamp_us = hal_micros();

    // 2. Apply calibration offset bias
    // apply_calibration_bias();

    // 3. Rotate to correct orientation
    // imu_apply_orientation_rotation();
    //imu_raw.gyro_x *= IMU_ORIENTATION_X;
    //imu_raw.gyro_y *= IMU_ORIENTATION_Y;
    //imu_raw.gyro_z *= IMU_ORIENTATION_Z;
}

static void remove_bias_from_imu_reading(imu_reading_t* imu_no_bias, const imu_reading_t* imu_raw, const imu_reading_t* imu_bias) {
    imu_no_bias->acc_x  = imu_raw->acc_x  - imu_bias->acc_x;
    imu_no_bias->acc_y  = imu_raw->acc_y  - imu_bias->acc_y;
    imu_no_bias->acc_z  = imu_raw->acc_z  - imu_bias->acc_z;
    imu_no_bias->gyro_x = imu_raw->gyro_x - imu_bias->gyro_x;
    imu_no_bias->gyro_y = imu_raw->gyro_y - imu_bias->gyro_y;
    imu_no_bias->gyro_z = imu_raw->gyro_z - imu_bias->gyro_z;
}


void imu_filter(imu_reading_t* filtered, const imu_reading_t* raw)
{
    // TODO
    /*

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
    */
}

static void print_calib()
{
    gcs_printf(MAV_SEVERITY_DEBUG, "IMU Calibration done, samples: %d, bias: Gx: %f, Gy: %f, Gz: %f, Ax: %f, Ay: %f, Az: %f",
        CALIBRATION_SAMPLES,
        imu_calib.gyro_x,
        imu_calib.gyro_y,
        imu_calib.gyro_z,
        imu_calib.acc_x,
        imu_calib.acc_y,
        imu_calib.acc_z
    );
}

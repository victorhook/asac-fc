#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/****************** LED INDICATOR CODES ******************/
/* slow blink - waiting for aircraft to be stationary    */
/* fast blink - calibrating (do not move)                */

/********** ACCELEROMETER CALIBRATION **********/
#define MPU6050_ACCELX_LEVEL 79.40
#define MPU6050_ACCELY_LEVEL -103.94
#define MPU6050_ACCELZ_LEVEL 1225.42

/********** INVERT AXES **********/
#define MPU6050_INVERT_ROLL
/*#define MPU6050_INVERT_PITCH*/
/*#define MPU6050_INVERT_YAW*/

/********** IMU CALIBRATION ROUTINE SETTINGS **********/
#define MPU6050_CAL_REST_TIME 1500
#define MPU6050_CAL_MIN_ACCEL 200
#define MPU6050_CAL_READINGS 1000

/*
 * When enabled, the sensor does not have to be level during calibration.
 * Instead, the net acceleration vector will be used to determine initial
 * orientation.
 */
#define MPU6050_CAL_GRAVITY_ZERO
/*
 * When enabled, acceleration data will be used to wait for rest before
 * starting the calibration routine in calibrate().
 */
#define MPU6050_CAL_WAIT_FOR_REST

/********** MPU6050 I2C AND REGISTER ADDRESSES **********/
#define MPU6050_I2C_ADDRESS 0x68
#define MPU6050_REG_POWER_MANAGEMENT 0x6B
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

/* units: microseconds */
#define MPU6050_I2C_TIMEOUT_PRE_BYTE 150

/********** MPU6050 POWER SETTINGS **********/
#define MPU6050_COMMAND_POWER_ON 0b00000000

/********** MPU6050 GYROSCOPE ACCURACY SETTINGS **********/
/*#define MPU6050_GYRO_ACCURACY 0b00000000*/     /* +/-  250 deg/sec */
#define MPU6050_GYRO_ACCURACY 0b00001000       /* +/-  500 deg/sec */
/*#define MPU6050_GYRO_ACCURACY 0b00010000*/     /* +/- 1000 deg/sec */
/*#define MPU6050_GYRO_ACCURACY 0b00011000*/     /* +/- 2000 deg/sec */

/********** MPU6050 ACCELEROMETER ACCURACY SETTINGS **********/
/*#define MPU6050_ACCEL_ACCURACY 0b00000000*/       /* +/-  2 g's */
/*#define MPU6050_ACCEL_ACCURACY 0b00001000*/       /* +/-  4 g's */
#define MPU6050_ACCEL_ACCURACY 0b00010000       /* +/-  8 g's */
/*#define MPU6050_ACCEL_ACCURACY 0b00011000*/       /* +/- 16 g's */

/********** CONSTANTS FOR MATH **********/
#define MPU6050_RADIANS_PER_DEGREE 0.01745329f
#define MPU6050_DEGREES_PER_TICK 0.0152672f
#define MPU6050_TICKS_PER_G 4096


struct quaternion {
    float w;
    float x;
    float y;
    float z;
};

typedef struct quaternion quaternion_t;

struct vector {
    float x;
    float y;
    float z;
};

typedef struct vector vector_t;

/*
* object for containing mpu6050 raw sensor data
*/
struct mpu6050_data {
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
};

/*
* type for containing mpu6050 raw sensor data
*/
typedef struct mpu6050_data mpu6050_data_t;

/*
* object for encapsulating mpu6050 state
*/
struct mpu6050_inst {
    float x_zero, y_zero, z_zero;

    quaternion_t orientation;

    absolute_time_t timer;

    mpu6050_data_t data;

    i2c_inst_t *i2c;

    uint led_pin;

    uint8_t start;
};

/*
 * type for encapsulating mpu6050 state
 */
typedef struct mpu6050_inst mpu6050_inst_t;

/*
 * Initialize mpu6050 object. Pass 0 for argument pin to disable status led.
 * Returns 0 if initialization is successfull.
 * Returns 1 if there is no response on i2c bus.
 */
int mpu6050_init(mpu6050_inst_t *inst, i2c_inst_t *i2c, uint pin);

/*
 * Returns the average result of n sensor readings
 * Returns 0 if reading was successfull.
 * Returns 1 if there is no response on i2c bus.
 */
int mpu6050_avg_reading(mpu6050_inst_t* inst, mpu6050_data_t* data, uint16_t n);

/*
 * Reads sensors and updates orientation.
 * call between 50Hz and 250Hz for best results.
 * Returns 0 if successfull.
 * Returns 1 if there is no response on i2c bus.
 */
int mpu6050_update_state(mpu6050_inst_t *inst);

/*
 * Returns the roll angle in degrees
 * -180 < roll < 180
 */
float mpu6050_get_roll(const mpu6050_inst_t *inst);

/*
 * Returns the roll pitch in degrees
 * -90 < pitch < 90
 */
float mpu6050_get_pitch(const mpu6050_inst_t *inst);

/*
 * Returns the yaw angle in degrees
 * -180 < yaw < 180
 */
float mpu6050_get_yaw(const mpu6050_inst_t *inst);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __MPU6050_H__ */
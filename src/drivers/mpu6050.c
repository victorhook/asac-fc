#include "mpu6050.h"

/*
 * returns the magnitude of vector v
 */
static float vector_norm(const vector_t* v) {
    return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

/*
 * returns the magnitude of quaternion q
 */
static float quaternion_norm(const quaternion_t* q) {
    return sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
}

/*
 * returns the product of two unit quaternions
 */
static quaternion_t quaternion_product(const quaternion_t* p, const quaternion_t* q) {
    quaternion_t result = {
        .w = p->w * q->w - p->x * q->x - p->y * q->y - p->z * q->z,
        .x = p->w * q->x + p->x * q->w + p->y * q->z - p->z * q->y,
        .y = p->w * q->y - p->x * q->z + p->y * q->w + p->z * q->x,
        .z = p->w * q->z + p->x * q->y - p->y * q->x + p->z * q->w
    };

    /* scale for noise reduction */
    float l = quaternion_norm(&result);

    result.w /= l;
    result.x /= l;
    result.y /= l;
    result.x /= l;

    return result;
}

/*
 * Configures mpu6050 power management and sensor accuracy.
 * Returns 0 if successfull
 * Returns 1 if there is no i2c response
 */
static int mpu6050_config(mpu6050_inst_t* inst) {
    uint8_t buffer[2];

    int ret;

    /* configure power management */
    buffer[0] = MPU6050_REG_POWER_MANAGEMENT;
    buffer[1] = MPU6050_COMMAND_POWER_ON;
    ret = i2c_write_timeout_us(
        inst->i2c,
        MPU6050_I2C_ADDRESS,
        buffer, 2,
        false,
        MPU6050_I2C_TIMEOUT_PRE_BYTE * 2
    );
    if (ret != 2) return 1;

    /* configure gyro accuracy */
    buffer[0] = MPU6050_REG_GYRO_CONFIG;
    buffer[1] = MPU6050_GYRO_ACCURACY;
    ret = i2c_write_timeout_us(
        inst->i2c,
        MPU6050_I2C_ADDRESS,
        buffer, 2,
        false,
        MPU6050_I2C_TIMEOUT_PRE_BYTE * 2
    );
    if (ret != 2) return 1;

    /* configure accelerometer accuracy */
    buffer[0] = MPU6050_REG_ACCEL_CONFIG;
    buffer[1] = MPU6050_ACCEL_ACCURACY;
    ret = i2c_write_timeout_us(
        inst->i2c,
        MPU6050_I2C_ADDRESS,
        buffer, 2,
        false,
        MPU6050_I2C_TIMEOUT_PRE_BYTE * 2
    );
    if (ret != 2) return 1;

    return 0;
}

/*
 * gets the most recent measurements from the sensors and
 * stores them in mpu6050 object inst.
 * Returns 0 if successfull
 * Returns 1 if there is no response on i2c bus
 */
static int mpu6050_fetch(mpu6050_inst_t* inst) {
    uint8_t buffer[14];

    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;

    int ret;

    ret = i2c_write_timeout_us(
        inst->i2c,
        MPU6050_I2C_ADDRESS,
        &reg, 1,
        true,
        MPU6050_I2C_TIMEOUT_PRE_BYTE
    );
    if (ret != 1) return 1;

    ret = i2c_read_timeout_us(
        inst->i2c,
        MPU6050_I2C_ADDRESS,
        buffer, 14,
        false,
        MPU6050_I2C_TIMEOUT_PRE_BYTE * 14
    );
    if (ret != 14) return 1;

    inst->data.accel_x = buffer[0] << 8 | buffer[1];
    inst->data.accel_y = buffer[2] << 8 | buffer[3];
    inst->data.accel_z = buffer[4] << 8 | buffer[5];
    inst->data.temp = buffer[6] << 8 | buffer[7];
    inst->data.gyro_x = buffer[8] << 8 | buffer[9];
    inst->data.gyro_y = buffer[10] << 8 | buffer[11];
    inst->data.gyro_z = buffer[12] << 8 | buffer[13];

    return 0;
}

/*
 * blocks until the acceleration measurements are not changing
 * more than MPU6050_CAL_MIN_ACCEL.
 * Returns 0 if successfull
 * Returns 1 if there is no reponse on i2c bus
 */
static int mpu6050_wait_for_rest(mpu6050_inst_t* inst) {
    uint8_t led_state = 0;

    uint16_t count = 0;
    uint16_t rest = 0;

    int32_t x_prev, y_prev, z_prev;

    absolute_time_t timer = get_absolute_time();

    while (rest < MPU6050_CAL_REST_TIME) {
        while (absolute_time_diff_us(timer, get_absolute_time()) < 4000);
        timer = get_absolute_time();

        if (mpu6050_fetch(inst))
            return 1;

        int32_t x_diff = abs(inst->data.accel_x - x_prev);
        int32_t y_diff = abs(inst->data.accel_y - y_prev);
        int32_t z_diff = abs(inst->data.accel_z - z_prev);

        x_prev = inst->data.accel_x;
        y_prev = inst->data.accel_y;
        z_prev = inst->data.accel_z;

        if (x_diff < MPU6050_CAL_MIN_ACCEL &&
            y_diff < MPU6050_CAL_MIN_ACCEL &&
            z_diff < MPU6050_CAL_MIN_ACCEL)
            ++rest;
        else 
            rest = 0;

        if (inst->led_pin) {
            if (++count % 200 == 0)
                led_state = !led_state;
            gpio_put(inst->led_pin, led_state);
        }
    }
    return 0;
}

/*
 * Initialize mpu6050 object. Pass 0 for argument pin to disable status led.
 * Returns 0 if initialization is successfull.
 * Returns 1 if there is no response on i2c bus.
 */
int mpu6050_init(mpu6050_inst_t* inst, i2c_inst_t* i2c, uint pin) {
    inst->led_pin = pin;

    inst->i2c = i2c;

    inst->start = 1;

    inst->orientation.w = 0.70710;
    inst->orientation.x = 0.70710;
    inst->orientation.y = 0.00001;
    inst->orientation.z = 0.00001;

    if (mpu6050_config(inst))
        return 1;

    #ifdef MPU6050_CAL_WAIT_FOR_REST
    if (mpu6050_wait_for_rest(inst))
        return 1;
    #endif /* MPU6050_CAL_WAIT_FOR_REST */

    mpu6050_data_t calibration_data;
    if (mpu6050_avg_reading(inst, &calibration_data, MPU6050_CAL_READINGS))
        return 1;

    inst->x_zero = calibration_data.gyro_x;
    inst->y_zero = calibration_data.gyro_y;
    inst->z_zero = calibration_data.gyro_z;

    float accel_x = calibration_data.accel_x - MPU6050_ACCELX_LEVEL;
    float accel_y = calibration_data.accel_y - MPU6050_ACCELY_LEVEL;
    float accel_z = calibration_data.accel_z - MPU6050_ACCELZ_LEVEL;

    vector_t accel_net = {
        .x = accel_x,
        .y = accel_y,
        .z = accel_z
    };

    float angle_x_accel = asin(accel_y / vector_norm(&accel_net));
    float angle_y_accel = asin(accel_x / vector_norm(&accel_net));

    angle_x_accel /= MPU6050_RADIANS_PER_DEGREE;
    angle_y_accel /= -MPU6050_RADIANS_PER_DEGREE;

    #ifdef MPU6050_CAL_GRAVITY_ZERO

    quaternion_t initial_roll = {
        .w = cos(angle_x_accel * MPU6050_RADIANS_PER_DEGREE * 0.5), 
        .x = sin(angle_x_accel * MPU6050_RADIANS_PER_DEGREE * 0.5),
        .y = 0.0001,
        .z = 0.0001
    };

    inst->orientation = quaternion_product(&inst->orientation, &initial_roll);

    quaternion_t initial_pitch = {
        .w = cos(angle_y_accel * MPU6050_RADIANS_PER_DEGREE * 0.5), 
        .x = 0.0001, 
        .y = sin(angle_y_accel * MPU6050_RADIANS_PER_DEGREE * 0.5), 
        .z = 0.0001 
    };

    inst->orientation = quaternion_product(&inst->orientation, &initial_pitch);

    #endif /* MPU6050_CAL_GRAVITY_ZERO */

    gpio_put(inst->led_pin, 0);

    return 0;
}

/*
 * Returns the average result of n sensor readings
 * Returns 0 if reading was successfull.
 * Returns 1 if there is no response on i2c bus.
 */
int mpu6050_avg_reading(mpu6050_inst_t* inst, mpu6050_data_t* data, uint16_t n) {
    float gyro_x = 0;
    float gyro_y = 0;
    float gyro_z = 0;

    float accel_x = 0;
    float accel_y = 0;
    float accel_z = 0;

    absolute_time_t timer = get_absolute_time();

    uint16_t count = 0;

    uint8_t led_state = 0;

    for (uint16_t i = 0; i < n; ++i) {
        while (absolute_time_diff_us(timer, get_absolute_time()) < 4000);
        timer = get_absolute_time();

        if (mpu6050_fetch(inst))
            return 1;

        gyro_x += inst->data.gyro_x / (float)n;
        gyro_y += inst->data.gyro_y / (float)n;
        gyro_z += inst->data.gyro_z / (float)n;

        accel_x += inst->data.accel_x / (float)n;
        accel_y += inst->data.accel_y / (float)n;
        accel_z += inst->data.accel_z / (float)n;

        if (inst->led_pin) {
            if (++count % 25 == 0)
                led_state = !led_state;
            gpio_put(inst->led_pin, led_state); 
        }
    }

    data->gyro_x = gyro_x;
    data->gyro_y = gyro_y;
    data->gyro_z = gyro_z;

    data->accel_x = accel_x;
    data->accel_y = accel_y;
    data->accel_z = accel_z;

    return 0;
}

/*
 * Reads sensors and updates orientation.
 * call between 50Hz and 250Hz for best results.
 * Returns 0 if successfull.
 * Returns 1 if there is no response on i2c bus.
 */
int mpu6050_update_state(mpu6050_inst_t* inst) {
    if (mpu6050_fetch(inst))
        return 1;

    if (inst->start)  {
        inst->start = 0;
        inst->timer = get_absolute_time();
    } else {
        /* units: seconds */
        float t_delta = absolute_time_diff_us(inst->timer, get_absolute_time());
              t_delta /= 1000000.0;
        inst->timer = get_absolute_time();

        vector_t w = {
            .x = (inst->data.gyro_x - inst->x_zero),
            .y = (inst->data.gyro_y - inst->y_zero),
            .z = (inst->data.gyro_z - inst->z_zero)
        };

        w.x *= MPU6050_DEGREES_PER_TICK * MPU6050_RADIANS_PER_DEGREE;
        w.y *= MPU6050_DEGREES_PER_TICK * MPU6050_RADIANS_PER_DEGREE;
        w.z *= MPU6050_DEGREES_PER_TICK * MPU6050_RADIANS_PER_DEGREE;;

        float w_norm = vector_norm(&w);

        if (w_norm == 0)
            w_norm = 0.00001f;

        quaternion_t rotation = {
            .w = cos((t_delta * w_norm) / 2),
            .x = (sin((t_delta * w_norm) / 2) * w.x) / w_norm,
            .y = (sin((t_delta * w_norm) / 2) * w.y) / w_norm,
            .z = (sin((t_delta * w_norm) / 2) * w.z) / w_norm
        };

        inst->orientation = quaternion_product(&inst->orientation, &rotation);
    }
    return 0;
}

/*
 * Returns the roll angle in degrees
 * -180 < roll < 180
 */
float mpu6050_get_roll(const mpu6050_inst_t* inst) {
    float result = atan2(
        2 * inst->orientation.x * inst->orientation.w -
        2 * inst->orientation.y * inst->orientation.z,
        1 - 2 * inst->orientation.x * inst->orientation.x -
        2 * inst->orientation.z * inst->orientation.z
    );

    result /= MPU6050_RADIANS_PER_DEGREE;

    if (result < -90) 
        result += 270;
    else 
        result -= 90;

    #ifdef MPU6050_INVERT_ROLL
    result *= -1;
    #endif /* MPU6050_INVERT_ROLL */

    return result;
}

/*
 * Returns the pitch angle in degrees
 * -90 < pitch < 90
 */
float mpu6050_get_pitch(const mpu6050_inst_t* inst) {
    float result = asin(
        2 * inst->orientation.x * inst->orientation.y +
        2 * inst->orientation.z * inst->orientation.w
    );

    result /= MPU6050_RADIANS_PER_DEGREE;

    #ifdef MPU6050_INVERT_PITCH
    result *= -1;
    #endif /* MPU6050_INVERT_PITCH */

    return result;
}

/*
 * Returns the yaw angle in degrees
 * -180 < yaw < 180
 */
float mpu6050_get_yaw(const mpu6050_inst_t* inst) {
    float result = atan2(
        2 * inst->orientation.y * inst->orientation.w -
        2 * inst->orientation.x * inst->orientation.z,
        1 - 2 * inst->orientation.y * inst->orientation.y -
        2 * inst->orientation.z * inst->orientation.z
    );

    result /= MPU6050_RADIANS_PER_DEGREE;

    #ifdef MPU6050_INVERT_YAW
    result *= -1;
    #endif /* MPU6050_INVERT_YAW */

    return result;
}
#include "param.h"

#include "hal.h"
#include "serial.h"
#include "util/flightmode.h"
#include "mavlink_driver/mavlink_driver.h"

// -- Motor -- //
float mot_pwm_min;
float mot_pwm_max;
float mot_pwm_arm;

// -- Flight modes and channels -- //
float fltmode_channel;
float fltmode1;
float fltmode2;
float fltmode3;
float arm_channel;
float arm_pwm;
float roll_channel;
float pitch_channel;
float yaw_channel;
float throttle_channel;

// -- Tuning -- //
float atc_rat_rll_p;
float atc_rat_rll_i;
float atc_rat_rll_d;
float atc_rat_rll_ff;
float atc_rat_rll_imax;
float atc_rat_pit_p;
float atc_rat_pit_i;
float atc_rat_pit_d;
float atc_rat_pit_ff;
float atc_rat_pit_imax;
float atc_rat_yaw_p;
float atc_rat_yaw_i;
float atc_rat_yaw_d;
float atc_rat_yaw_ff;
float atc_rat_yaw_imax;

// -- Rate -- //
float max_roll_rate;
float max_pitch_rate;
float max_yaw_rate;

// -- IMU -- //
float imu_calib_gyro_on_boot;
float imu_offset_x;
float imu_offset_y;
float imu_offset_z;
float imu_accelcal_x;
float imu_accelcal_x;
float imu_accelcal_x;

// -- Hardware - Board specific -- //
// I2C 2
float brd_i2c1_sda;
float brd_i2c1_scl;
float brd_i2c1_freq;

// I2C 1
float brd_i2c2_sda;
float brd_i2c2_scl;
float brd_i2c2_freq;

// SPI 1
float brd_spi1_mosi;
float brd_spi1_miso;
float brd_spi1_clk;
float brd_spi1_freq;

// SPI 2
float brd_spi2_mosi;
float brd_spi2_miso;
float brd_spi2_clk;
float brd_spi2_freq;

// Serial 1
float brd_serial1_rx;
float brd_serial1_tx;
float brd_serial1_baud;
float brd_serial1_protocol;

// Serial 2
float brd_serial2_rx;
float brd_serial2_tx;
float brd_serial2_baud;
float brd_serial2_protocol;

// LEDs
float brd_led1;
float brd_led2;
float brd_led3;

// IMU
float brd_imu_type;
float brd_imu_bus;
float brd_imu_ss;

// Scheduler
float sched_loop_rate;


mav_param_t mav_params[] = {
    // Motor
    {"MOT_PWM_MIN", &mot_pwm_min},
    {"MOT_PWM_MAX", &mot_pwm_max},
    {"MOT_PWM_ARM", &mot_pwm_arm},

    // Flight modes and channels
    {"FLTMODE_CH", &fltmode_channel},
    {"FLTMODE1",   &fltmode1},
    {"FLTMODE2",   &fltmode2},
    {"FLTMODE3",   &fltmode3},
    {"ARM_CH",     &arm_channel},
    {"ARM_PWM",    &arm_pwm},
    {"RLL_CH",     &roll_channel},
    {"PIT_CH",    &pitch_channel},
    {"YAW_CH",    &yaw_channel},
    {"THR_CH",    &throttle_channel},

    // Tuning
    {"ATC_RAT_RLL_P",    &atc_rat_rll_p},
    {"ATC_RAT_RLL_I",    &atc_rat_rll_i},
    {"ATC_RAT_RLL_D",    &atc_rat_rll_d},
    {"ATC_RAT_RLL_FF",    &atc_rat_rll_ff},
    {"ATC_RAT_RLL_IMAX", &atc_rat_rll_imax},

    {"ATC_RAT_PIT_P",    &atc_rat_pit_p},
    {"ATC_RAT_PIT_I",    &atc_rat_pit_i},
    {"ATC_RAT_PIT_D",    &atc_rat_pit_d},
    {"ATC_RAT_PIT_FF",    &atc_rat_pit_ff},
    {"ATC_RAT_PIT_IMAX", &atc_rat_pit_imax},

    {"ATC_RAT_YAW_P",    &atc_rat_yaw_p},
    {"ATC_RAT_YAW_I",    &atc_rat_yaw_i},
    {"ATC_RAT_YAW_D",    &atc_rat_yaw_d},
    {"ATC_RAT_YAW_FF",    &atc_rat_yaw_ff},
    {"ATC_RAT_YAW_IMAX", &atc_rat_yaw_imax},

    // IMU
    {"INS_GYR_CAL",   &imu_calib_gyro_on_boot},
    {"INS_GYROFFS_X", &imu_offset_x},
    {"INS_GYROFFS_Y", &imu_offset_y},
    {"INS_GYROFFS_Z", &imu_offset_z},
    {"INS_ACCSCAL_X", &imu_accelcal_x},
    {"INS_ACCSCAL_Y", &imu_accelcal_x},
    {"INS_ACCSCAL_Z", &imu_accelcal_x},

    // Rate
    {"ATC_RAT_RLL_MAX", &max_roll_rate},
    {"ATC_RAT_PIT_MAX", &max_pitch_rate},
    {"ATC_RAT_YAW_MAX", &max_yaw_rate},

    // Hardware - Board specific
    {"BRD_I2C1_SDA",  &brd_i2c1_sda},
    {"BRD_I2C1_SCL",  &brd_i2c1_scl},
    {"BRD_I2C1_FREQ", &brd_i2c1_freq},

    {"BRD_I2C2_SDA",  &brd_i2c2_sda},
    {"BRD_I2C2_SCL",  &brd_i2c2_scl},
    {"BRD_I2C2_FREQ", &brd_i2c2_freq},

    {"BRD_SPI1_MOSI", &brd_spi1_mosi},
    {"BRD_SPI1_MISO", &brd_spi1_miso},
    {"BRD_SPI1_CLK",  &brd_spi1_clk},
    {"BRD_SPI1_FREQ", &brd_spi1_freq},

    {"BRD_SPI2_MOSI", &brd_spi2_mosi},
    {"BRD_SPI2_MISO", &brd_spi2_miso},
    {"BRD_SPI2_CLK",  &brd_spi2_clk},
    {"BRD_SPI2_FREQ", &brd_spi2_freq},

    {"BRD_SERIAL1_RX",   &brd_serial1_rx},
    {"BRD_SERIAL1_TX",   &brd_serial1_tx},
    {"BRD_SERIAL1_BAUD", &brd_serial1_baud},
    {"BRD_SERIAL1_PROT", &brd_serial1_protocol},

    {"BRD_SERIAL2_RX",   &brd_serial2_rx},
    {"BRD_SERIAL2_TX",   &brd_serial2_tx},
    {"BRD_SERIAL2_BAUD", &brd_serial2_baud},
    {"BRD_SERIAL2_PROT", &brd_serial2_protocol},

    {"BRD_LED1",      &brd_led1},
    {"BRD_LED2",      &brd_led2},
    {"BRD_LED3",      &brd_led3},

    {"BRD_IMU_TYPE",  &brd_imu_type}, // Type of IMU, options are: [1=MPU6050, 2=BMI270]
    {"BRD_IMU_BUS",   &brd_imu_bus},  // Which bus the IMU talks on, options are: [1=i2c1, 2=i2c2, 3=spi1, 4=spi2]
    {"BRD_IMU_SS",    &brd_imu_ss},

    // Scheduler
    {"SCHED_LOOP_RATE",    &sched_loop_rate} // Loop rate of scheduler, in Hz
};

const uint16_t nbr_of_parameters = (sizeof(mav_params) / sizeof(mav_param_t));

void reset_to_default_parameters()
{
    // MOTOR
    mot_pwm_min = 1000;
    mot_pwm_max = 2000;
    mot_pwm_arm = 1175;

    // Flight modes and channels (0-based)
    fltmode_channel = 5;
    fltmode1 = FLIGHTMODE_ACRO; // Only support ACRO atm :)
    fltmode2 = FLIGHTMODE_ACRO; // Only support ACRO atm :)
    fltmode3 = FLIGHTMODE_ACRO; // Only support ACRO atm :)
    arm_channel = 4;
    arm_pwm = 1500;
    roll_channel = 0;
    pitch_channel = 1;
    yaw_channel = 3;
    throttle_channel = 2;

    // Tuning - These are divided by 100 when used by the PIDs
    atc_rat_rll_p = 0.01;
    atc_rat_rll_i = 0.01;
    atc_rat_rll_d = 0;
    atc_rat_rll_ff = 0;
    atc_rat_rll_imax = 1;
    atc_rat_pit_p = 0.01;
    atc_rat_pit_i = 0.01;
    atc_rat_pit_d = 0;
    atc_rat_pit_ff = 0;
    atc_rat_pit_imax = 1;
    atc_rat_yaw_p = 0.1;
    atc_rat_yaw_i = 0.1;
    atc_rat_yaw_d = 0;
    atc_rat_yaw_ff = 0;
    atc_rat_yaw_imax = 1;

    // IMU
    imu_calib_gyro_on_boot = 1;
    imu_offset_x = 0;
    imu_offset_y = 0;
    imu_offset_z = 0;
    imu_accelcal_x = 0;
    imu_accelcal_x = 0;
    imu_accelcal_x = 0;

    // Rates
    max_roll_rate = 720;
    max_pitch_rate = 720;
    max_yaw_rate = 360;
    
    // Hardware - Board specific
    brd_serial1_baud = 115200;
    brd_serial1_protocol = SERIAL_PROTOCOL_CRSF;
    brd_serial2_baud = 115200;
    brd_serial2_protocol = SERIAL_PROTOCOL_MAVLINK;

    // Scheduler
    sched_loop_rate = 1000;
}

void read_parameters()
{
    float params_eeprom[nbr_of_parameters];
    uint32_t param_size = sizeof(params_eeprom);
    uint32_t crc;
    
    if (!hal_read_param(param_size, &crc, (uint8_t*) &params_eeprom))
    {
        gcs_printf(0, "Failed to read parameters from eeprom, using default values\n");
        // Reset parameter to default values, then write then to eeprom
        reset_to_default_parameters();
        write_parameters();

        if (!hal_read_param(param_size, &crc, (uint8_t*) &params_eeprom))
        {
            gcs_printf(0, "Still failed to read params... Something is off!\n");
            return;
        }
    }
    
    for (int i = 0; i < nbr_of_parameters; i++)
    {
        *mav_params[i].value = params_eeprom[i];
    }

    for (int i = 0; i < nbr_of_parameters; i++)
    {
        mav_param_t* param = &mav_params[i];
        printf(" %s: %f\n", param->id, *param->value);
    }
}

void write_parameters()
{
    float eeprom_params[nbr_of_parameters];

    // Fill eeprom param array with proper values
    for (int i = 0; i < nbr_of_parameters; i++)
    {
        eeprom_params[i] = *((float*) mav_params[i].value);
    }
    
    uint32_t param_size = sizeof(eeprom_params);
    uint32_t crc = 0;
    // Calculate CRC for entire buffer array
    uint8_t* buf = (uint8_t*) eeprom_params;
    for (int i = 0; i < sizeof(eeprom_params); i++)
    {
        crc += buf[i];
    }

    hal_write_param(param_size, crc, buf);
}

bool set_param_value(const char* param_id, const float param_value, const uint8_t param_type)
{
    for (int i = 0; i < nbr_of_parameters; i++)
    {
        mav_param_t* param = &mav_params[i];
        if (strncmp(param_id, param->id, 16) == 0)
        {
            *param->value = param_value;
            return true;
        }
    }
    return false;
}

float get_param_value(const uint16_t index)
{
    if (index >= nbr_of_parameters) return -1.0;

    return *mav_params[index].value;
}

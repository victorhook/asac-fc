#ifndef ASAC_FC_H
#define ASAC_FC_H

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"

#ifdef LINUX
    // We run in linux environment if only testing, eg unit tests etc
#else
    #include <pico/stdlib.h>
    #include <pico/stdio.h>
    #include <pico/time.h>
#endif


#define DEBUG_MODE

#ifdef DEBUG_MODE
    #define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

// #define TELEMETRY_LOGGING

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define HIGH 1
#define LOW  0

//#define RP2040

/*
    The PID controller expects the following orientation of the IMU:
        - X: Forward
        - Y: Right
        - Z: Down
    If the IMU is NOT mounted in this direction, please adjust the signs below.
*/
#define IMU_ORIENTATION_X  1
#define IMU_ORIENTATION_Y  1
#define IMU_ORIENTATION_Z -1


// #define IMU_MPU_6050_I2C
#define IMU_BMI270_SPI

void system_init();

void system_reboot();

uint32_t us_since_boot();

uint32_t ms_since_boot();


#endif /* ASAC_FC_H */

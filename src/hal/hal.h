
#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "serial.h"
#include "spi.h"
#include "i2c.h"
#include "pwm.h"


typedef enum
{
    BUS_TYPE_I2C    = 1,
    BUS_TYPE_SPI    = 2,
    BUS_TYPE_SERIAL = 3,
    BUS_TYPE_SITL   = 4
} bus_type_t;


typedef enum
{
    HAL_GPIO_FUNCTION_OUTPUT,
    HAL_GPIO_FUNCTION_INPUT,
    HAL_GPIO_FUNCTION_INPUT_PULLUP
} hal_gpio_function_t;

#define HAL_GPIO_HIGH 1
#define HAL_GPIO_LOW  0

#define MAX_PARAMS_STORAGE_DATA_SIZE 4096

/** Initializes primary HAL stuff */
int hal_init();

int hal_do_init();

void hal_reboot();

// -- ADC -- //

int hal_adc_init(const uint8_t pin);

void hal_adc_read(const int channel, int* value);

// -- GPIO -- //

/** Initializes the GPIO with given `pin` to the `function` with default value `default_value` */
int hal_gpio_init(const uint8_t pin, const hal_gpio_function_t function, const uint8_t default_value);

void hal_gpio_set(const uint8_t pin, const uint8_t value);

// -- Log -- //

uint32_t hal_log_get_next_id();

bool hal_log_init(const uint32_t log_id);

uint32_t hal_log_write(const uint8_t* data, const uint16_t size);


bool hal_write_param(const uint32_t param_size, const uint32_t crc, const uint8_t* buf);

bool hal_read_param(uint32_t* param_size, uint32_t* crc, uint8_t* buf);


int hal_pin_set(const uint8_t port, const uint8_t pin, const uint8_t value);

void hal_sleep_us(const uint32_t us);

void hal_sleep_ms(const uint32_t ms);

uint32_t hal_millis();

uint32_t hal_micros();

void hal_pre_pid_loop();

void hal_post_pid_loop();

bool usb_connected();


bool get_serial_with_protocol(serial_t* serial, const serial_protocol_t protocol);


// Bus defines
extern i2c_t hal_i2c1;
extern i2c_t hal_i2c2;

extern spi_t hal_spi1;
extern spi_t hal_spi2;

extern serial_t hal_serial0;
extern serial_t hal_serial1;
extern serial_t hal_serial2;


#endif
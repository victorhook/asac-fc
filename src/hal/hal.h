
#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "serial.h"
#include "spi.h"
#include "i2c.h"


// Pull in correct HAL
#if defined(HAL_RP2040)
    #warning "Using RP2040 HAL"
    #include "hal_rp2040/hal_impl.h"
#elif defined(HAL_SITL)
    #warning "Using SITL HAL"
    #include "hal_rp2040/hal_impl.h"
#endif


typedef enum
{
    BUS_TYPE_I2C  = 1,
    BUS_TYPE_SPI  = 2,
    BUS_TYPE_SITL = 3
} bus_type_t;

typedef struct
{
    bus_type_t bus;
    union
    {
        bus_config_i2c_t i2c;
        bus_config_spi_t spi;
    };
} bus_config_t;


typedef enum
{
    HAL_GPIO_FUNCTION_OUTPUT,
    HAL_GPIO_FUNCTION_INPUT,
    HAL_GPIO_FUNCTION_INPUT_PULLUP
} hal_gpio_function_t;

#define HAL_GPIO_HIGH 1
#define HAL_GPIO_LOW  0

/** Initializes primary HAL stuff */
int hal_init();

int hal_do_init();

// -- ADC -- //

int hal_adc_init(const int channel);

void hal_adc_read(const int channel, int* value);

// -- GPIO -- //
int hal_gpio_init(const uint8_t pin, const hal_gpio_function_t function, const uint8_t value);


uint32_t hal_log_get_next_id();

bool hal_log_init(const uint32_t log_id);

uint32_t hal_log_write(const uint8_t* data, const uint16_t size);


bool hal_write_param(const uint32_t param_size, const uint32_t crc, const uint8_t* buf);

bool hal_read_param(uint32_t param_size, uint32_t* crc, uint8_t* buf);


int hal_pwm_set(const int channel, const int gpio, const uint16_t value);

int hal_pin_set(const uint8_t port, const uint8_t pin, const uint8_t value);

void hal_sleep_us(const uint32_t us);

void hal_sleep_ms(const uint32_t ms);

uint32_t hal_millis();

uint32_t hal_micros();

void hal_pre_pid_loop();

void hal_post_pid_loop();

bool usb_connected();


void system_init();

void system_reboot();


// Bus defines
extern i2c_t i2c1;
extern i2c_t i2c2;

extern spi_t spi1;
extern spi_t spi2;

extern serial_t serial0;
extern serial_t serial1;


#endif

#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "serial/serial.h"

// Pull in correct HAL
#if defined(HAL_RP2040)
    #warning "Using RP2040 HAL"
    #include "hal_rp2040/hal_impl.h"
#elif defined(HAL_SITL)
    #warning "Using SITL HAL"
    #include "hal_rp2040/hal_impl.h"
#endif



/** Initializes primary HAL stuff */
int hal_init();

// -- ADC //

bool hal_adc_init(const int channel);

void hal_adc_read(const int channel, int* value);


/** Initializes the serial port with given number and baudrate */
int hal_serial_init(serial_t* serial, const uint8_t serial_nbr, const uint32_t baudrate);

/** Writes data to the serial port (non-blocking). Returns the number of bytes written. */
int hal_serial_write(serial_t* serial, const uint8_t* data, const uint16_t len);

/** Reads number of bytes into `data`. Returns the number of bytes read. */
int hal_serial_read(serial_t* serial, uint8_t* data, const uint16_t len);

/** Returns number of bytes available in the serial RX buffer */
int hal_serial_available(const uint8_t serial_nbr);


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



#endif

#ifndef I2C_H
#define I2C_H

#include "util/ringbuf.h"

#define I2C_RX_BUFF_SIZE 256
#define I2C_TX_BUFF_SIZE 256

typedef struct
{
    uint8_t nbr;
    ringbuf_t rx_buf;
    ringbuf_t tx_buf;
} i2c_t;

typedef struct
{
    uint8_t nbr;
    uint8_t sda;
    uint8_t scl;
    uint32_t freq;
} bus_config_i2c_t;

int hal_i2c_init(const bus_config_i2c_t config);

int hal_i2c_read(i2c_t* i2c, const uint8_t addr, uint8_t *data, const uint32_t len);

int hal_i2c_read_byte(i2c_t* i2c, const uint8_t addr, uint8_t *byte);

int hal_i2c_write(i2c_t* i2c, const uint8_t addr, const uint8_t *data, const uint32_t len);

int hal_i2c_write_byte(i2c_t* i2c, const uint8_t addr, const uint8_t *data, const uint32_t len);

#endif
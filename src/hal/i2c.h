
#ifndef I2C_H
#define I2C_H

#include "util/ringbuf.h"

#define I2C_RX_BUFF_SIZE 256
#define I2C_TX_BUFF_SIZE 256

typedef struct
{
    uint8_t  sda;
    uint8_t  scl;
    uint32_t freq;
} bus_config_i2c_t;

typedef struct
{
    uint8_t          nbr;
    ringbuf_t        rx_buf;
    ringbuf_t        tx_buf;
    bus_config_i2c_t config;
    void*            ctx;
    bool             initialized;
} i2c_t;


int hal_i2c_init(i2c_t* i2c);

uint8_t hal_i2c_read_reg(i2c_t* i2c, const uint8_t addr, const uint8_t reg);

int hal_i2c_read(i2c_t* i2c, const uint8_t addr, const uint8_t reg, uint8_t *data, const uint32_t len);

uint8_t hal_i2c_read_byte(i2c_t* i2c);

int hal_i2c_write(i2c_t* i2c, const uint8_t addr, const uint8_t *data, const uint32_t len);

int hal_i2c_write_reg(i2c_t* i2c, const uint8_t addr, const uint8_t reg, const uint8_t data);

void hal_i2c_probe_bus(const uint8_t bus, uint8_t devices[20], uint8_t* devices_found);

bool hal_i2c_probe(const i2c_t* i2c, uint8_t addr);

#endif
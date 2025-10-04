#include "hal.h"
#include <hardware/gpio.h>
#include <hardware/i2c.h>


int hal_i2c_init(i2c_t* i2c)
{
    if (i2c->nbr == 1)
    {
        i2c->ctx = i2c0;
    }
    else if (i2c->nbr == 2)
    {
        i2c->ctx = i2c1;
    }
    else
    {
        return -1;
    }

    i2c_init(i2c->ctx, i2c->config.freq);
    
    gpio_set_function(i2c->config.sda, GPIO_FUNC_I2C);
    gpio_set_function(i2c->config.scl, GPIO_FUNC_I2C);

    gpio_pull_up(i2c->config.scl);
    gpio_pull_up(i2c->config.sda);

    i2c->initialized = true;

    return 0;
}

uint8_t hal_i2c_read_reg(i2c_t* i2c, const uint8_t addr)
{
    uint8_t value;
    i2c_write_blocking(i2c->ctx, addr, &addr, 1, false);
    i2c_read_blocking(i2c->ctx, addr, &value, 1, false);
    return value;
}

int hal_i2c_read(i2c_t* i2c, const uint8_t addr, uint8_t *data, const uint32_t len)
{
    return i2c_read_blocking(i2c->ctx, addr, data, len, false);
}

uint8_t hal_i2c_read_byte(i2c_t* i2c)
{
    return i2c_read_byte_raw(i2c->ctx);
}

int hal_i2c_write(i2c_t* i2c, const uint8_t addr, const uint8_t *data, const uint32_t len)
{
    return i2c_write_blocking(i2c->ctx, addr, data, len, false);
}

int hal_i2c_write_reg(i2c_t* i2c, const uint8_t addr, const uint8_t data)
{
    return i2c_write_blocking(i2c->ctx, addr, &data, 1, false);
}

bool hal_i2c_probe(const i2c_t* i2c, uint8_t addr)
{
    uint8_t dummy;
    return i2c_read_blocking(i2c->ctx, addr, &dummy, 1, false) != -1;
}
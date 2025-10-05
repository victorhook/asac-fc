#include "hal.h"
#include "i2c.h"

int hal_i2c_init(i2c_t* i2c)
{
    return 0;
}

int hal_i2c_read(i2c_t* i2c, const uint8_t addr, const uint8_t reg, uint8_t *data, const uint32_t len)
{
    return 0;
}

uint8_t hal_i2c_read_reg(i2c_t* i2c, const uint8_t addr, const uint8_t reg)
{
    return 0;
}

uint8_t hal_i2c_read_byte(i2c_t* i2c) 
{
    return 0;
}

int hal_i2c_write(i2c_t* i2c, const uint8_t addr, const uint8_t *data, const uint32_t len)
{
    return 0;
}

int hal_i2c_write_byte(i2c_t* i2c, const uint8_t addr, const uint8_t *data, const uint32_t len)
{
    return 0;
}

int hal_i2c_write_reg(i2c_t* i2c, const uint8_t addr, const uint8_t reg, const uint8_t data)
{
    return 0;
}

bool hal_i2c_probe(const i2c_t* i2c, uint8_t addr)
{
    return false;
}
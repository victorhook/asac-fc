#include "hal.h"
#include "i2c.h"

int hal_i2c_init(i2c_t* i2c)
{
    return 0;
}

int hal_i2c_read(i2c_t* i2c, const uint8_t addr, uint8_t *data, const uint32_t len)
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

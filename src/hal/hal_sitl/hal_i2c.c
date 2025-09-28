#include "hal.h"

int hal_i2c_init(const bus_config_i2c_t config)
{
    return 0;
}

int hal_i2c_read(i2c_t* i2c, const uint8_t addr, uint8_t *data, const uint32_t len)
{
    return 0;
}

int hal_i2c_read_byte(i2c_t* i2c, const uint8_t addr, uint8_t *byte)
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

i2c_t i2c1;
i2c_t i2c2;
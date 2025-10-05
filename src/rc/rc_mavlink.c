#include "rc_mavlink.h"


int rc_mavlink_init(serial_t* serial)
{
    return 0;
}

bool rc_mavlink_parse_byte(const uint8_t data, rc_input_t* rc_input)
{
    return true;
}


uint16_t rc_mavlink_scale_channel(const uint16_t raw)
{
    return raw;
}
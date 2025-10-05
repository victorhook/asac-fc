
#ifndef RC_MAVLINK_H
#define RC_MAVLINK_H

#include "rc.h"


int rc_mavlink_init(serial_t* serial);

bool rc_mavlink_parse_byte(const uint8_t data, rc_input_t* rc_input);

uint16_t rc_mavlink_scale_channel(const uint16_t raw);


#endif
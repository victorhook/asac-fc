#include "rc_mavlink.h"
#include "mavlink_driver.h"


static bool new_packet = false;


void on_channels_override(const mavlink_rc_channels_override_t* msg)
{
    rc_input_raw.channels[0] = msg->chan1_raw;
    rc_input_raw.channels[1] = msg->chan2_raw;
    rc_input_raw.channels[2] = msg->chan3_raw;
    rc_input_raw.channels[3] = msg->chan4_raw;
    rc_input_raw.channels[4] = msg->chan5_raw;
    rc_input_raw.channels[5] = msg->chan6_raw;
    rc_input_raw.channels[6] = msg->chan7_raw;
    rc_input_raw.channels[7] = msg->chan8_raw;
    rc_input_raw.channels[8] = msg->chan9_raw;
    rc_input_raw.channels[9] = msg->chan10_raw;
    rc_input_raw.channels[10] = msg->chan11_raw;
    rc_input_raw.channels[11] = msg->chan12_raw;
    rc_input_raw.channels[12] = msg->chan13_raw;
    rc_input_raw.channels[13] = msg->chan14_raw;
    rc_input_raw.channels[14] = msg->chan15_raw;
    rc_input_raw.channels[15] = msg->chan16_raw;
    rc_input_raw.channels[16] = msg->chan17_raw;
    rc_input_raw.channels[17] = msg->chan18_raw;
    new_packet = true;
}

int rc_mavlink_init(serial_t* serial)
{
    return mavlink_subscribe_to_rc_channels_override(on_channels_override) ? 0 : -1;
}

bool rc_mavlink_parse_byte(const uint8_t data, rc_input_t* rc_input)
{
    return new_packet;
}


uint16_t rc_mavlink_scale_channel(const uint16_t raw)
{
    return raw;
}
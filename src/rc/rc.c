#include "rc.h"
#include "ibus.h"
#include "crsf.h"
#include "mavlink_driver/mavlink_driver.h"
#include <string.h>

// TODO: Fix where this should come from
rc_config_t config =
{
    .protocol = RC_PROTOCOL_ELRS
};

typedef int (*rc_do_init)();
typedef bool (*rc_parse_byte)(const uint8_t, rc_input_t*);
typedef uint16_t (*rc_do_scale_channel)(const uint16_t);

int dummy_init() { return 0; }
bool dummy_parse_byte(const uint8_t byte, rc_input_t* rc_input) { return false; }
uint16_t dummy_scale_channel(const uint16_t raw) { return raw; }

typedef struct
{
    rc_do_init          init;
    rc_parse_byte       parse_byte;
    rc_do_scale_channel scale;
} rc_handler_t;


// -- RC Implementations -- //
rc_handler_t rc_ibus =
{
    .init = ibus_init,
    .parse_byte = ibus_parse_byte,
    .scale = ibus_scale_channel
};

rc_handler_t rc_crsf =
{
    .init = crsf_init,
    .parse_byte = crsf_parse_byte,
    .scale = crsf_scale_channel
};

rc_handler_t rc_dummy =
{
    .init = dummy_init,
    .parse_byte = dummy_parse_byte,
    .scale = dummy_scale_channel
};


rc_handler_t* rc_handler;


int rc_init() {
    switch (config.protocol)
    {
        case RC_PROTOCOL_IBUS:
            memcpy(rc_handler, &rc_ibus, sizeof(rc_handler_t));
            break;
        case RC_PROTOCOL_ELRS:
            memcpy(rc_handler, &rc_crsf, sizeof(rc_handler_t));
            break;
        default:
            memcpy(rc_handler, &rc_dummy, sizeof(rc_handler_t));
            gcs_printf(MAV_SEVERITY_WARNING, "Invalid RC protocol %d", config.protocol);
            return -1;
    }

    return rc_handler->init();
}

void rc_update(rc_input_t* rc_input)
{
    // TODO
    //return rc_handler->parse_byte
}

uint16_t receiver_scale_channel(const uint16_t raw)
{
    return rc_handler->scale(raw);
}



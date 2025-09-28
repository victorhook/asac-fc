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

static int dummy_init() { return 0; }
static bool dummy_parse_byte(const uint8_t byte, rc_input_t* rc_input) { return false; }
static uint16_t dummy_scale_channel(const uint16_t raw) { return raw; }

typedef struct
{
    rc_do_init          init;
    rc_parse_byte       parse_byte;
    rc_do_scale_channel scale;
} backend_t;


static backend_t backend;


int rc_init() {
    switch (config.protocol)
    {
        case RC_PROTOCOL_IBUS:
            backend.init = ibus_init;
            backend.parse_byte = ibus_parse_byte;
            backend.scale = ibus_scale_channel;
            break;
        case RC_PROTOCOL_ELRS:
            backend.init = crsf_init;
            backend.parse_byte = crsf_parse_byte;
            backend.scale = crsf_scale_channel;
            break;
        default:
            backend.init = dummy_init;
            backend.parse_byte = dummy_parse_byte;
            backend.scale = dummy_scale_channel;
            gcs_printf(MAV_SEVERITY_WARNING, "Invalid RC protocol %d", config.protocol);
            return -1;
    }

    return backend.init();
}

void rc_update(rc_input_t* rc_input)
{
    // TODO
    //return rc_handler->parse_byte
}

uint16_t receiver_scale_channel(const uint16_t raw)
{
    return backend.scale(raw);
}



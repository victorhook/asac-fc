#include "rc.h"
#include "ibus.h"
#include "crsf.h"
#include "mavlink_driver/mavlink_driver.h"

#include "hal.h"
#include "serial.h"


typedef int (*rc_do_init)(serial_t* serial);
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

rc_input_t rc_input_raw;
rc_input_t rc_input_scaled;
sensor_t rc_sensor;

extern float rc_protocol;
extern float rc_timeout;
extern float mot_pwm_min;
static uint32_t last_packet = 0;


int rc_init()
{
    serial_t* serial;
    bool serial_found = false;

    rc_sensor.present = false;
    rc_sensor.enabled = true;
    rc_sensor.healthy = false;

    switch ((rc_protocol_t) rc_protocol)
    {
        case RC_PROTOCOL_IBUS:
            backend.init = ibus_init;
            backend.parse_byte = ibus_parse_byte;
            backend.scale = ibus_scale_channel;
            break;
        case RC_PROTOCOL_ELRS:
            serial_found = true;
            serial_found = get_serial_with_protocol(serial, SERIAL_PROTOCOL_CRSF);
            backend.init = crsf_init;
            backend.parse_byte = crsf_parse_byte;
            backend.scale = crsf_scale_channel;
            break;
        default:
            rc_sensor.enabled = false;
            backend.init = dummy_init;
            backend.parse_byte = dummy_parse_byte;
            backend.scale = dummy_scale_channel;
            gcs_printf(MAV_SEVERITY_WARNING, "Invalid RC protocol %d", (int) rc_protocol);
            return -1;
    }

    if (!serial_found)
    {
        gcs_printf(MAV_SEVERITY_WARNING, "Failed to find serial port for RC protocol %d", (int) rc_protocol);
        return -1;
    }

    int res = backend.init(serial);

    if (res)
    {
        rc_sensor.present = true;
    }
    return res;
}

void rc_update()
{
    // TODO
    //return rc_handler->parse_byte
    rc_sensor.healthy = ((hal_millis() - last_packet) > rc_timeout);
}

uint16_t receiver_scale_channel(const uint16_t raw)
{
    return backend.scale(raw);
}

bool is_rc_connected()
{
    return ((hal_millis() - rc_input_scaled.timestamp) < ((uint32_t) rc_timeout));
}

uint16_t rc_get_channel(const uint8_t channel)
{
    if (channel > RC_MAX_NBR_OF_CHANNELS) return mot_pwm_min;
    return rc_input_scaled.channels[channel];
}


/*

static void rc_constrain(rc_input_t* constrained, const rc_input_t* unconstrained) {
    // TODO: Move this to receiver!
    //crsf_scale_rc_channels(unconstrained, constrained);
    for (int i = 0; i < RC_MAX_NBR_OF_CHANNELS ; i++) {
        uint16_t rc_scaled = receiver_scale_channel(unconstrained->channels[i]);
        constrained->channels[i] = constrain(rc_scaled, 1000, 2000);
    }
}

*/
#include "rc.h"
#include "ibus.h"
#include "crsf.h"
#include "rc_mavlink.h"
#include "mavlink_driver/mavlink_driver.h"

#include "hal.h"
#include "serial.h"
#include "util.h"


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

rc_input_t rc_input_raw = {0};
rc_input_t rc_input_scaled = {0};
sensor_t rc_sensor;

extern float rc_protocol;
extern float rc_timeout;
extern float rc_max;
extern float rc_mid;
extern float rc_min;
extern float rc_dz;

extern float roll_channel;
extern float pitch_channel;
extern float yaw_channel;
extern float throttle_channel;

static uint32_t last_packet = 0;

static void safe_set_channel(rc_input_t* input, const int channel, const uint16_t value)
{
    if ((channel < 0) || (channel >= RC_MAX_NBR_OF_CHANNELS)) return;
    input->channels[channel] = value;
}

int rc_init()
{
    serial_t* serial;
    bool serial_found = false;

    for (int i = 0; i < RC_MAX_NBR_OF_CHANNELS; i++)
    {
        rc_input_scaled.channels[i] = rc_min;
    }

    safe_set_channel(&rc_input_raw, roll_channel, rc_mid);
    safe_set_channel(&rc_input_raw, pitch_channel, rc_mid);
    safe_set_channel(&rc_input_raw, yaw_channel, rc_mid);
    safe_set_channel(&rc_input_raw, throttle_channel, rc_min);

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
    rc_sensor.healthy = ((hal_millis() - last_packet) < rc_timeout);

    if (rc_sensor.healthy)
    {
        int dz_low = rc_min + dz_low;
        int dz_high = rc_max - dz_low;
        int dz_mid_low = rc_mid - dz_low;
        int dz_mid_high = rc_mid - dz_high;

        for (int i = 0; i < RC_MAX_NBR_OF_CHANNELS; i++)
        {
            // Scale from raw input to "rx_min - rc_max"
            int scaled = backend.scale(rc_input_raw.channels[i]);

            // Constrain to ensure channel values are within boundaries
            scaled = constrain(scaled, rc_min, rc_max);

            // Check if close to deadzone
            if (scaled < dz_low)
            {   // Within deadzone at lower end
                scaled = (int) rc_min;
            }
            else if (scaled > dz_high)
            {   // Within deadzone at higher end
                scaled = (int) rc_max;
            }
            else if ( (scaled > dz_mid_low) && (scaled < dz_mid_high) && (i != throttle_channel) )
            {   // WIthin deadzone at middle. NOT for throttle though as we want this smooth.
                // If we add other flight modes such as althold in future, we DO want it for throttle as well though
                scaled = (int) rc_mid;
            }

            rc_input_scaled.channels[i] = scaled;
        }

        // TODO: Should this be moved to different struct?
        rc_input_scaled.link_quality = rc_input_raw.link_quality;
        rc_input_scaled.rssi         = rc_input_raw.rssi;
        rc_input_scaled.timestamp    = rc_input_raw.timestamp;
    }
    else
    {
        // If RC is not healthy, we'll just set control channels to default
        safe_set_channel(&rc_input_scaled, roll_channel,     rc_mid);
        safe_set_channel(&rc_input_scaled, pitch_channel,    rc_mid);
        safe_set_channel(&rc_input_scaled, yaw_channel,      rc_mid);
        safe_set_channel(&rc_input_scaled, throttle_channel, rc_min);
    }
}

bool is_rc_connected()
{
    return ((hal_millis() - rc_input_scaled.timestamp) < ((uint32_t) rc_timeout));
}

uint16_t rc_get_channel(const uint8_t channel)
{
    if (channel > RC_MAX_NBR_OF_CHANNELS) return rc_min;
    return rc_input_scaled.channels[channel];
}


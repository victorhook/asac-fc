#ifndef RC_H
#define RC_H

#include "hal.h"
#include "sensor.h"
#include "stdint.h"
#include <stdbool.h>

#define RC_MAX_NBR_OF_CHANNELS 18

typedef struct
{
    uint32_t timestamp;
    uint16_t channels[RC_MAX_NBR_OF_CHANNELS];
    int rssi;          // dBm
    int link_quality;  // %
} rc_input_t;

typedef enum
{
    RC_PROTOCOL_ELRS = 1,
    RC_PROTOCOL_IBUS = 2,
    RC_PROTOCOL_MAVLINK = 3
} rc_protocol_t;

typedef int (*rc_do_update)(rc_input_t* rc_input);

typedef struct
{
    rc_protocol_t protocol;
    rc_do_update update;
} rc_config_t;


/*
 * Initializes the receiver.
 * Depending on which RX protocol is found in the settings, this
 * will initialize the correct RX protocol handler, eg IBUS/CRSF etc.
 * Returns 0 on success.
 */
int rc_init();

/*
 * Fills `rc_input` with the curent state of the receiver
 * This state includes the latest received packet as well as statistics of the RX link.
 */
void rc_update();


bool is_rc_connected();


uint16_t rc_get_channel(const uint8_t channel);


extern sensor_t rc_sensor;
extern rc_input_t rc_input_raw;
extern rc_input_t rc_input_scaled;


#endif /* RC_H */

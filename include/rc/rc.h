#ifndef RC_H
#define RC_H

#include "stdint.h"

#define RC_MAX_NBR_OF_CHANNELS 16

typedef struct
{
    uint32_t timestamp;
    uint16_t channels[RC_MAX_NBR_OF_CHANNELS];
} rc_input_t;

typedef struct
{
    int rssi;          // dBm
    int link_quality;  // %
} rc_link_statistics_t;

typedef struct
{
    rc_input_t           last_packet;
    rc_link_statistics_t statistics;
} rx_state_t;


/*
 * Parses a single byte in the RX state machine.
 * Returns true if a new packet has been parsed
 */
typedef bool (*rc_parse_byte)(const uint8_t byte);

/*
 * Fills `state` with the current RX state.
 */
typedef void (*rc_get_last_state)(rx_state_t* state);


/*
 * Scales the value of a single RAW channel
 * Returns the scaled channel value
 */
typedef uint16_t (*rc_scale_channel)(const uint16_t raw);


#endif /* RC_H */

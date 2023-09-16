#ifndef IBUS_H
#define IBUS_H

#include <stdio.h>
#include "pico/stdlib.h"

#include "rc/rc.h"


typedef struct {
    uint16_t channels[14];
}__attribute__((packed)) ibus_packet_t;

typedef struct {
    uint32_t successful_packets;
    uint32_t parse_errors;
    uint32_t last_received_packet;
    uint32_t packet_rate;
} ibus_statistics_t;


/*
 * Initializes the IBUS state machine.
 * Returns 0 on success.
 */
int ibus_init();

/*
 * Processes a single byte in the internal state machine.
 */
bool ibus_parse_byte(uint8_t data);

/*
 * Fills the `state` with the current state of the IBUS receiver.
 */
void ibus_get_last_state(rx_state_t* state);


uint16_t ibus_scale_channel(const uint16_t raw);


#endif /* IBUS_H */

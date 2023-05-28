#ifndef IBUS_H
#define IBUS_H

#include <stdio.h>
#include "pico/stdlib.h"

typedef struct {
    uint32_t timestamp;  // In ms
    uint16_t channels[14];
}__attribute__((packed)) ibus_packet_t;

typedef struct {
    uint32_t successful_packets;
    uint32_t parse_errors;
    uint32_t last_received_packet;
    uint32_t packet_rate;
} ibus_statistics_t;


uint32_t getCurrentTime();

/*
 * Returns statistics of ibus state machine
 */
void ibus_get_statistics(ibus_statistics_t* statistics);

/*
 * Processes a single byte in the internal state machine.
*/
void ibus_process_byte(uint8_t data);

/*
 * Fills the packet pointer with the data of the last received IBUS packet.
 * Note that if no packed has been received, the timestamp on the packet
 * will be 0.
*/
void ibus_get_last_packet(ibus_packet_t* packet);


#endif /* IBUS_H */

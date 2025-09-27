#ifndef IBUS_H
#define IBUS_H

#include "rc.h"

/*
 * Initializes the IBUS state machine.
 * Returns 0 on success.
 */
int ibus_init();

/*
 * Processes a single byte in the internal state machine.
 */
bool ibus_parse_byte(const uint8_t byte, rc_input_t* rc_input);


uint16_t ibus_scale_channel(const uint16_t raw);


#endif /* IBUS_H */

#ifndef CRSF_H
#define CRSF_H

#include "rc.h"

/*
 * Initializes the CRSF state machine.
 * Returns 0 on success.
 */
int crsf_init();

/*
 * Parses a single byte in the CRSF state machine.
 * This method returns true if a new packet has been detected
 */
bool crsf_parse_byte(const uint8_t byte, rc_input_t* rc_input);


uint16_t crsf_scale_channel(const uint16_t raw);


#endif /* CRSF_H */

#ifndef ASAC_FC_H
#define ASAC_FC_H

#include "pico/stdlib.h"

#define DEBUG_MODE

#ifdef DEBUG_MODE
    #define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

#define TELEMETRY_LOGGING

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//#define RP2040

uint64_t us_since_boot();


#endif /* ASAC_FC_H */

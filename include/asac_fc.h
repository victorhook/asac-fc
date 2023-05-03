#ifndef ASAC_FC_H
#define ASAC_FC_H


#define DEBUG_MODE

#ifdef DEBUG_MODE
    #define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

#define TELEMETRY_LOGGING 0

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//#define RP2040


#endif /* ASAC_FC_H */

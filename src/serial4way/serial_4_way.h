#ifndef SERIAL_4_WAY_H
#define SERIAL_4_WAY_H


#include "stdint.h"


typedef struct {
    uint8_t start;
    uint8_t command;
    uint16_t address;
    uint8_t param_len; 
    uint8_t param[255];
    uint16_t crc;
} pc_commant_t;


#endif /* SERIAL_4_WAY_H */


#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>


typedef struct
{
    uint16_t voltage_mv;
    uint16_t current_ma;
    uint8_t  perc;
    uint32_t capacity_used_mah;
} battery_t;

void battery_update();

extern battery_t bat1;


#endif
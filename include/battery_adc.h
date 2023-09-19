#ifndef BATTERY_H
#define BATTERY_H

#include "hardware/adc.h"


typedef struct
{
    uint16_t raw;
    float scaledMv;
} vbat_t;


/* Initializes the battery ADC. */
int battery_adc_init();


/* Reads the battery voltage and returns as float. */
float battery_adc_read();


#endif /* BATTERY_H */

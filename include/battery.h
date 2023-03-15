#ifndef BATTERY_H
#define BATTERY_H

#include "hardware/adc.h"


/* Initializes the battery ADC. */
void battery_measure_init();


/* Reads the battery voltage and returns as float. */
float battery_measure_read();


#endif /* BATTERY_H */

#ifndef BATTERY_ADC_H
#define BATTERY_ADC_H


/* Initializes the battery ADC. */
int battery_adc_init();

/* Reads the battery voltage and returns as float. */
float battery_adc_read();


#endif /* BATTERY_ADC_H */

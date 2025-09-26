#ifndef LED_H
#define LED_H

#include "stdint.h"

#define LED_RED   0 // PIN_LED_RED
#define LED_GREEN 0 // PIN_LED_GREEN
#define LED_BLUE  0 // PIN_LED_BLUE


int led_init();


void led_set(uint8_t pin, uint8_t value);


void led_run_boot_sequence();


#endif /* LED_H */

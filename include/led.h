#ifndef LED_H
#define LED_H

#include "stdint.h"
#include "machine.h"

#define LED_RED   PIN_LED_RED
#define LED_GREEN PIN_LED_GREEN
#define LED_BLUE  PIN_LED_BLUE


int led_init();


void led_set(uint8_t pin, uint8_t value);


void led_run_boot_sequence();


#endif /* LED_H */

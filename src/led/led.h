#ifndef LED_H
#define LED_H

#include "stdint.h"

extern float brd_led1;
extern float brd_led2;
extern float brd_led3;

int led_init();

void led_on(const uint8_t pin);

void led_off(const uint8_t pin);

void led_blink(const uint8_t pin, const uint16_t on_time, const uint16_t off_time);

void led_run_boot_sequence();

static inline void led1_on() { led_on(brd_led1); }
static inline void led1_off() { led_off(brd_led1); }

static inline void led2_on() { led_on(brd_led2); }
static inline void led2_off() { led_off(brd_led2); }

static inline void led3_on() { led_on(brd_led3); }
static inline void led3_off() { led_off(brd_led3); }

#endif /* LED_H */

#ifndef SERIAL_MAVLINK_H
#define SERIAL_MAVLINK_H

#include "motor.h"


/* Initializes the serial mavlink handler */
int serial_mavlink_init();

/*
 * Checks for any received mavlink messages through USB and
 * responds to them accordingly.
 */
void serial_mavlink_update();


// Global variable. This is used to set motor throttle from mavlink
extern motor_command_t motor_command_test;


#endif /* SERIAL_MAVLINK_H */

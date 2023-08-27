#ifndef SERIAL_MAVLINK_H
#define SERIAL_MAVLINK_H


/* Initializes the serial mavlink handler */
int serial_mavlink_init();

/*
 * Checks for any received mavlink messages through USB and
 * responds to them accordingly.
 */
void serial_mavlink_update();


#endif /* SERIAL_MAVLINK_H */

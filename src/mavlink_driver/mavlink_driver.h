#ifndef mavlink_driver_H
#define mavlink_driver_H

#include "mavlink.h"
#include "motor/motor.h"
#include "serial.h"

#define MAVLINK_CHANNEL_SERIAL 0
#define MAVLINK_SYSTEM_ID      1
#define MAVLINK_COMPONENT_ID   1

typedef struct
{
    serial_t* serial;
    uint8_t channel;
} mavlink_channel_handler_t;

/* Initializes the serial mavlink handler */
int mavlink_driver_init();

/*
 * Checks for any received mavlink messages through USB and
 * responds to them accordingly.
 */
void mavlink_driver_update();

// -- Subscriptions -- //
typedef void (*on_rc_channels_override_fn)(const mavlink_rc_channels_override_t*);
bool mavlink_subscribe_to_rc_channels_override(on_rc_channels_override_fn fn);


// Global variable. This is used to set motor throttle from mavlink
extern motor_output_t motor_command_test;


void gcs_printf(const uint8_t severity, const char* fmt, ...);

extern mavlink_channel_handler_t gcs_handler;

#endif /* mavlink_driver_H */

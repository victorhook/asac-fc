#include "serial_mavlink.h"
#include "state.h"

#include "mavlink/asac/mavlink.h"

#include <tusb.h>
#include <pico/stdio.h>
#include <pico/stdio_usb.h>


#define MAVLINK_MAX_BUF_SIZE 256
#define MAVLINK_CHANNEL_SERIAL 0
#define MAVLINK_SYSTEM_ID 0

static mavlink_message_t msg_rx;
static mavlink_message_t msg_tx;
static uint8_t buf_rx[MAVLINK_MAX_BUF_SIZE];
static uint8_t buf_tx[MAVLINK_MAX_BUF_SIZE];
static mavlink_status_t status;


#define MAVLINK_SERIAL_WRITE(buf, size) \
    tud_cdc_write(buf, size); \
    tud_cdc_write_flush()


int serial_mavlink_init()
{
    tud_cdc_read_flush();
    tud_cdc_write_flush();
    return 0;
}


void serial_mavlink_broadcast_heartbeat()
{
    if (!tud_cdc_connected()) {
        return;
    }

    mavlink_msg_heartbeat_pack_chan(
        MAVLINK_SYSTEM_ID,
        MAV_COMP_ID_ALL,
        MAVLINK_CHANNEL_SERIAL,
        &msg_tx,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
            (state.can_run_motors ? MAV_MODE_FLAG_SAFETY_ARMED : 0),
        0,
        MAV_STATE_STANDBY
    );

    uint16_t size = mavlink_msg_to_send_buffer(buf_tx, &msg_tx);
    MAVLINK_SERIAL_WRITE(buf_tx, size);
}

static inline uint32_t serial_available() {
    // Manually update the TinyUSB task
    tud_task();
    // Check if we're connected and data available
    return tud_cdc_connected() && tud_cdc_available();
}

void serial_mavlink_update() {
    serial_mavlink_broadcast_heartbeat();
    return;

    // We'll consume this time MAXIMUM, so we don't consume too much CPU
    const uint16_t max_execution_time_us = 300;
    uint32_t t0 = time_us_32();

    while (serial_available() & ((time_us_32() - t0) < max_execution_time_us)) {
        char new_byte = tud_cdc_read_char();
        //tud_cdc_write(&c, 1);
        //tud_cdc_write_flush();
        if (mavlink_parse_char(new_byte, MAVLINK_CHANNEL_SERIAL, &msg_rx, &status)) {
            //switch (msg_rx.msgid) {
            //}
        }
    }
}
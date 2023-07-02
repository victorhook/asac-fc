#include "serial_mavlink.h"
#include "state.h"
#include "controller.h"
#include "asac_fc.h"
#include "mavlink_params.h"

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

typedef enum {
    PARAM_STATE_WAIT_FOR_REQUEST,
    PARAM_STATE_BROADCAST_VALUES
} param_state_t;

static param_state_t param_state;
static uint16_t current_param_index;



#define mavlink_write_serial(buf, size) \
    tud_cdc_write(buf, size); \
    tud_cdc_write_flush()


static inline void send_mavlink_msg(const mavlink_message_t* mav_msg) {
    uint16_t size = mavlink_msg_to_send_buffer(buf_tx, mav_msg);
    mavlink_write_serial(buf_tx, size);
}


/*
    Broadcasts MAVlink parameter values.
*/
bool broadcast_param_values();

void update_param_values();


int serial_mavlink_init()
{
    tud_cdc_read_flush();
    tud_cdc_write_flush();
    param_state = PARAM_STATE_WAIT_FOR_REQUEST;
    current_param_index = 0;
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
    mavlink_write_serial(buf_tx, size);
}

void serial_mavlink_send_raw_imu() {
    if (!tud_cdc_connected()) {
        return;
    }

    mavlink_msg_scaled_imu_pack_chan(
        MAVLINK_SYSTEM_ID,
        MAV_COMP_ID_IMU,
        MAVLINK_CHANNEL_SERIAL,
        &msg_tx,
        imu_raw.timestamp_us,
        (int16_t) (imu_raw.acc_x * 1000),
        (int16_t) (imu_raw.acc_y * 1000),
        (int16_t) (imu_raw.acc_z * 1000),
        (int16_t) (imu_raw.gyro_x * 1000),
        (int16_t) (imu_raw.gyro_y * 1000),
        (int16_t) (imu_raw.gyro_z * 1000),
        0, 0, 0
    );

    send_mavlink_msg(&msg_tx);
}

void serial_mavlink_send_attitude() {
    if (!tud_cdc_connected()) {
        return;
    }

    mavlink_msg_attitude_pack_chan(
        MAVLINK_SYSTEM_ID,
        MAV_COMP_ID_IMU,
        MAVLINK_CHANNEL_SERIAL,
        &msg_tx,
        ms_since_boot(),
        state.roll,
        state.pitch,
        state.yaw,
        state.roll_speed,
        state.pitch_speed,
        state.yaw_speed
    );

    send_mavlink_msg(&msg_tx);
}


static inline uint32_t serial_available() {
    // Manually update the TinyUSB task
    tud_task();
    // Check if we're connected and data available
    return tud_cdc_connected() && tud_cdc_available();
}

void serial_mavlink_update() {

    if (param_state == PARAM_STATE_BROADCAST_VALUES) {
        bool done_broadcasting = broadcast_param_values();
        if (done_broadcasting) {
            param_state = PARAM_STATE_WAIT_FOR_REQUEST;
            current_param_index = 0;
        }
    }

    // We'll consume this time MAXIMUM, so we don't consume too much CPU
    const uint16_t max_execution_time_us = 300;
    uint32_t t0 = time_us_32();

    static mavlink_command_int_t cmd_rx;
    static mavlink_command_ack_t cmd_ack;
    uint16_t msg_id;

    while (serial_available() & ((time_us_32() - t0) < max_execution_time_us)) {
        char new_byte = tud_cdc_read_char();
        //putchar(new_byte);
        //fflush(stdout);

        if (mavlink_parse_char(MAVLINK_CHANNEL_SERIAL, new_byte, &msg_rx, &status)) {
            //printf("MSG RX ID: %d\n", msg_rx.msgid);
            switch (msg_rx.msgid) {
                case MAVLINK_MSG_ID_COMMAND_INT:
                    // Command request
                    mavlink_msg_command_int_decode(&msg_rx, &cmd_rx);

                    switch (cmd_rx.command) {

                        case MAV_CMD_REQUEST_MESSAGE:

                            msg_id = (uint16_t) cmd_rx.param1;

                            switch (msg_id) {
                                case MAVLINK_MSG_ID_RAW_IMU:
                                    cmd_ack.command = MAVLINK_MSG_ID_RAW_IMU;
                                    cmd_ack.result = MAV_RESULT_ACCEPTED;
                                    mavlink_msg_command_ack_encode(
                                        MAVLINK_SYSTEM_ID,
                                        MAV_COMP_ID_IMU,
                                        &msg_tx,
                                        &cmd_ack
                                    );

                                    send_mavlink_msg(&msg_tx);
                                    break;
                                default:
                                    break;
                            }
                            break;
                        default:
                            break;
                    }
                    //mavlink_msg_command_int_encode()
                    break;
                case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                    update_param_values();
                    param_state = PARAM_STATE_BROADCAST_VALUES;
                    break;
            }
        }
    }
}


bool broadcast_param_values() {
    mavlink_msg_param_value_encode(
        MAVLINK_SYSTEM_ID,
        MAV_COMP_ID_ALL,
        &msg_tx,
        &((mavlink_param_value_t*) &system_params)[current_param_index]
    );
    send_mavlink_msg(&msg_tx);

    current_param_index++;
    return current_param_index >= NBR_OF_PARAM_VALUES;
}

void update_param_values() {
    pid_roll.p = system_params.pid_gyro_roll_p.param_value;
    pid_roll.i = system_params.pid_gyro_roll_i.param_value;
    pid_roll.d = system_params.pid_gyro_roll_d.param_value;

    pid_pitch.p = system_params.pid_gyro_pitch_p.param_value;
    pid_pitch.i = system_params.pid_gyro_pitch_i.param_value;
    pid_pitch.d = system_params.pid_gyro_pitch_d.param_value;

    pid_yaw.p = system_params.pid_gyro_yaw_p.param_value;
    pid_yaw.i = system_params.pid_gyro_yaw_i.param_value;
    pid_yaw.d = system_params.pid_gyro_yaw_d.param_value;
}
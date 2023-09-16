#include "serial_mavlink.h"
#include "state.h"
#include "controller.h"
#include "asac_fc.h"
#include "mavlink_params.h"
#include "settings.h"

#include "mavlink/common/mavlink.h"

#include <tusb.h>
#include <pico/stdio.h>
#include <pico/stdio_usb.h>


#define MAVLINK_MAX_BUF_SIZE   256
#define MAVLINK_CHANNEL_SERIAL 0
#define MAVLINK_SYSTEM_ID      0

// We'll consume this time MAXIMUM, so we don't consume too much CPU
#define UPDATE_FUNCTION_MAX_RUNTIME_US 300

#define mavlink_write_serial(buf, size) \
    tud_cdc_write(buf, size);           \
    tud_cdc_write_flush()


motor_command_t motor_command_test;

static mavlink_message_t msg_rx;
static mavlink_message_t msg_tx;
static uint8_t buf_tx[MAVLINK_MAX_BUF_SIZE];
static mavlink_status_t status;

// Periodic mavlink messages
#define HEARTBEAT_MSG_PERIOD_MS  1000
#define ATTITUDE_MSG_PERIOD_MS   100
#define RC_CHANNEL_MSG_PERIOD_MS 100
static uint32_t last_sent_heartbeat;
static uint32_t last_sent_attitude;
static uint32_t last_sent_rc_channels;

typedef enum {
    PARAM_STATE_WAIT_FOR_REQUEST,
    PARAM_STATE_BROADCAST_VALUES
} param_state_t;

static param_state_t param_state;
static uint16_t current_param_index;

// Mavlink message handlers handlers
static void handle_msg_command_int(const mavlink_message_t* msg_rx);
static void handle_msg_param_set(const mavlink_message_t* msg_rx);
static void handle_msg_param_request_list(const mavlink_message_t* msg_rx);

// Helper functions
static inline void send_mavlink_msg(const mavlink_message_t* mav_msg);
static inline uint32_t serial_available();

static bool broadcast_param_values();
static void update_param_values();
static bool set_param(const mavlink_param_set_t* msg_param_set);
static void broadcast_param_value(const mavlink_param_set_t* msg_param_set);

static void serial_mavlink_broadcast_heartbeat();
static void serial_mavlink_send_raw_imu();
static void serial_mavlink_send_attitude();
static void serial_mavlink_send_rc_channels();
static void serial_mavlink_statustext(const MAV_SEVERITY severity, const char* text);


int serial_mavlink_init()
{
    tud_cdc_read_flush();
    tud_cdc_write_flush();
    param_state = PARAM_STATE_WAIT_FOR_REQUEST;
    current_param_index = 0;
    last_sent_heartbeat = 0;
    return 0;
}


void serial_mavlink_update()
{
    if (!usb_connected())
    {
        // USB is not connected, so we don't care about wasting computation resources
        // on any type of checks here.
        return;
    }

    if (param_state == PARAM_STATE_BROADCAST_VALUES)
    {
        bool done_broadcasting = broadcast_param_values();
        if (done_broadcasting)
        {
            param_state = PARAM_STATE_WAIT_FOR_REQUEST;
            current_param_index = 0;
        }
    }

    uint32_t t0 = ms_since_boot();

    // Check if it's time for any periodic messages to be sent
    if ((t0 - last_sent_heartbeat) >= HEARTBEAT_MSG_PERIOD_MS)
    {
        serial_mavlink_broadcast_heartbeat();
        last_sent_heartbeat = t0;
    }
    if ((t0 - last_sent_attitude) >= ATTITUDE_MSG_PERIOD_MS)
    {
        serial_mavlink_send_attitude();
        last_sent_attitude = t0;
    }
    if ((t0 - last_sent_rc_channels) >= RC_CHANNEL_MSG_PERIOD_MS)
    {
        serial_mavlink_send_rc_channels();
        last_sent_rc_channels = t0;
    }

    uint16_t msg_id;
    t0 = us_since_boot();

    // Check for any input and read if there's any available
    while (serial_available() & ((us_since_boot() - t0) < UPDATE_FUNCTION_MAX_RUNTIME_US)) {
        char new_byte = tud_cdc_read_char();

        if (mavlink_parse_char(MAVLINK_CHANNEL_SERIAL, new_byte, &msg_rx, &status)) {
            // New mavlink message received, let's find a message handler
            //printf("MSG RX ID: %d\n", msg_rx.msgid);
            switch (msg_rx.msgid) {
                case MAVLINK_MSG_ID_COMMAND_INT:
                    handle_msg_command_int(&msg_rx);
                    break;
                case MAVLINK_MSG_ID_PARAM_SET:
                    handle_msg_param_set(&msg_rx);
                    break;
                case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                    handle_msg_param_request_list(&msg_rx);
                    break;
            }
        }
    }
}

// -- Private -- //

// Message handlers

static void handle_msg_command_int(const mavlink_message_t* msg_rx)
{
    const int REBOOT_AUTOPILOT = 1;
    mavlink_command_int_t mavlink_command;
    mavlink_msg_command_int_decode(msg_rx, &mavlink_command);

    uint8_t motor;
    uint8_t motor_test_throttle_type;
    float throttle;

    switch (mavlink_command.command) {
        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (mavlink_command.param1 == REBOOT_AUTOPILOT) {
                system_reboot();
            }
            break;
        case MAV_CMD_PREFLIGHT_STORAGE:
            switch ((PREFLIGHT_STORAGE_PARAMETER_ACTION) mavlink_command.param1) {
                case PARAM_READ_PERSISTENT:
                    break;
                case PARAM_WRITE_PERSISTENT:
                    settings_write_to_flash(&system_params);
                    break;
                case PARAM_RESET_CONFIG_DEFAULT:
                    break;
                case PARAM_RESET_SENSOR_DEFAULT:
                    break;
                case PARAM_RESET_ALL_DEFAULT:
                    break;
                default:
                    break;
            }
            break;
        case MAV_CMD_DO_MOTOR_TEST:
            motor = (uint8_t) mavlink_command.param1;
            motor_test_throttle_type = (uint8_t) mavlink_command.param2;
            if (motor_test_throttle_type != MOTOR_TEST_THROTTLE_PERCENT)
            {
                // TODO: Fix?
                // We only support throttle percentage
                return;
            }
            if ((motor < 1) || (motor > 4))
            {
                // TODO: Nbr of motors
                return;
            }

            // Set throttle to desired test throttle value
            // Incoming throttle is between 0-100, let's scale it to 0-1
            throttle = mavlink_command.param3 / 100.0;
            // "motor" is value between 1 and number of motors, so the index is motors-1
            ((float*) &motor_command_test)[motor - 1] = throttle;

            // Force arm!
            state.is_force_armed = true;

            break;
        default:
            break;
    }
}

static void handle_msg_param_set(const mavlink_message_t* msg_rx)
{
    mavlink_param_set_t msg_param_set;
    mavlink_msg_param_set_decode(msg_rx, &msg_param_set);
    if (set_param(&msg_param_set)) {
        broadcast_param_value(&msg_param_set);
    }
}

static void handle_msg_param_request_list(const mavlink_message_t* msg_rx)
{
    update_param_values();
    tud_cdc_write_flush();
    param_state = PARAM_STATE_BROADCAST_VALUES;
}

// Helpers

static void serial_mavlink_broadcast_heartbeat()
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

static void serial_mavlink_send_raw_imu() {
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

static void serial_mavlink_send_attitude() {
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

static void serial_mavlink_send_rc_channels() {
    mavlink_msg_rc_channels_pack_chan(
        MAVLINK_SYSTEM_ID,
        0,
        MAVLINK_CHANNEL_SERIAL,
        &msg_tx,
        ms_since_boot(),
        16,
        ctrl_rc_input_constrained.channels[0],
        ctrl_rc_input_constrained.channels[1],
        ctrl_rc_input_constrained.channels[2],
        ctrl_rc_input_constrained.channels[3],
        ctrl_rc_input_constrained.channels[4],
        ctrl_rc_input_constrained.channels[5],
        ctrl_rc_input_constrained.channels[6],
        ctrl_rc_input_constrained.channels[7],
        ctrl_rc_input_constrained.channels[8],
        ctrl_rc_input_constrained.channels[9],
        ctrl_rc_input_constrained.channels[10],
        ctrl_rc_input_constrained.channels[11],
        ctrl_rc_input_constrained.channels[12],
        ctrl_rc_input_constrained.channels[13],
        ctrl_rc_input_constrained.channels[14],
        ctrl_rc_input_constrained.channels[15],
        0xFFFF,
        0xFFFF,
        rx_state.statistics.rssi
    );
    send_mavlink_msg(&msg_tx);
}

static void serial_mavlink_statustext(const MAV_SEVERITY severity, const char* text) {
    mavlink_msg_statustext_pack_chan(
        MAVLINK_SYSTEM_ID,
        0,
        MAVLINK_CHANNEL_SERIAL,
        &msg_tx,
        severity,
        text
    );
    send_mavlink_msg(&msg_tx);
}

static bool broadcast_param_values() {
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

static bool set_param(const mavlink_param_set_t* msg_param_set) {
    for (int i = 0; i < NBR_OF_PARAM_VALUES; i++) {
        mavlink_param_value_t* param = &((mavlink_param_value_t*) &system_params)[i];
        if (strncmp(param->param_id, msg_param_set->param_id, 16) == 0) {
            param->param_value = msg_param_set->param_value;
            return true;
        }
    }
    return false;
}

static void broadcast_param_value(const mavlink_param_set_t* msg_param_set) {
    mavlink_param_value_t param = {
        .param_value = msg_param_set->param_value,
        .param_count = 0,
        .param_index = 0,
        .param_type = msg_param_set->param_type,
    };
    memcpy(param.param_id, msg_param_set->param_id, 16);

    mavlink_msg_param_value_encode(
        MAVLINK_SYSTEM_ID,
        MAV_COMP_ID_ALL,
        &msg_tx,
        &param
    );
    send_mavlink_msg(&msg_tx);
}

static void update_param_values() {
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

static void send_mavlink_msg(const mavlink_message_t* mav_msg)
{
    uint16_t size = mavlink_msg_to_send_buffer(buf_tx, mav_msg);
    mavlink_write_serial(buf_tx, size);
}

static uint32_t serial_available()
{
    // Manually update the TinyUSB task
    tud_task();
    // Check if we're connected and data available
    return tud_cdc_connected() && tud_cdc_available();
}

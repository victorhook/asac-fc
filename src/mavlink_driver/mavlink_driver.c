#include "mavlink_driver.h"
#include "serial.h"
#include "state.h"
#include "control/controller.h"
#include "util.h"
#include "param/param.h"
#include "util/flightmode.h"

#include <stdarg.h> // For printf
#include <stdio.h>

#include "mavlink.h"

mavlink_channel_handler_t gcs_handler;
motor_output_t motor_command_test;

// Periodic mavlink messages
#define HEARTBEAT_MSG_PERIOD_MS  1000
#define BATTERY_STATUS_PERIOD_MS 500
#define ATTITUDE_MSG_PERIOD_MS   100
#define RC_CHANNEL_MSG_PERIOD_MS 100
static uint32_t last_sent_heartbeat;
static uint32_t last_sent_battery_status;
static uint32_t last_sent_attitude;
static uint32_t last_sent_rc_channels;

#define MAVLINK_INTENRAL_BUF_SIZE 10

bool armed_force;
bool armed;
flightmode_t flightmode;

static bool send_param_request = false;
static uint32_t param_index = 0;

// Helper functions
static inline void send_mavlink_msg(const mavlink_message_t* mav_msg);

static void handle_mavlink_message(mavlink_message_t* msg, mavlink_status_t* status);


int mavlink_driver_init()
{
    last_sent_heartbeat      = 0;
    last_sent_battery_status = 0;
    last_sent_attitude       = 0;
    last_sent_rc_channels    = 0;

    // TODO: Separate this?
    gcs_handler.channel = 0;
    gcs_handler.serial = &serial0;

    return 0;
}


#define SEND_IF_TIME_FOR(function)

// -- Private -- //

// Message handlers

static void handle_rc_channels_override(mavlink_message_t* msg)
{
    mavlink_rc_channels_override_t rc_channels;
    mavlink_msg_rc_channels_override_decode(msg, &rc_channels);
    rc_input_t input =
    {
        rc_channels.chan1_raw,
        rc_channels.chan2_raw,
        rc_channels.chan3_raw,
        rc_channels.chan4_raw,
        rc_channels.chan5_raw,
        rc_channels.chan6_raw,
        rc_channels.chan7_raw,
        rc_channels.chan8_raw,
        rc_channels.chan9_raw,
        rc_channels.chan10_raw,
        rc_channels.chan11_raw,
        rc_channels.chan12_raw,
        rc_channels.chan13_raw,
        rc_channels.chan14_raw,
        rc_channels.chan15_raw,
        rc_channels.chan16_raw,
        rc_channels.chan17_raw,
        rc_channels.chan18_raw
    };
    input.timestamp = hal_millis();
    //rc_channels_override(&input);
}

// Helpers

uint32_t discarded_tx_packets = 0;

void mav_send(mavlink_channel_handler_t* channel, const mavlink_message_t* msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    if (hal_serial_write(channel->serial, buf, len) != len)
    {
        discarded_tx_packets++;
    }
}

void mav_send_heartbeat(mavlink_channel_handler_t* channel)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | ((armed) ? MAV_MODE_FLAG_SAFETY_ARMED : 0),
    flightmode,
MAV_STATE_ACTIVE
);
    mav_send(channel, &msg);
}


static void handle_command_int(mavlink_message_t* msg)
{
    mavlink_command_int_t cmd;
    mavlink_msg_command_int_decode(msg, &cmd);
    printf("COMMAND INT: %d\n", cmd.command);
    switch (cmd.command)
    {
        case MAV_CMD_DO_MOTOR_TEST:
            /*motor = (uint8_t) mavlink_command.param1;
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
            state.is_force_armed = true;*/
            break;

    }
}

static void handle_command_long(mavlink_message_t* msg)
{
    mavlink_command_int_t cmd;
    mavlink_msg_command_int_decode(msg, &cmd);
    //printf("COMMAND LONG: %d\n", cmd.command);
    switch (cmd.command)
    {
        case MAV_CMD_COMPONENT_ARM_DISARM:
            armed_force = cmd.param1 == 1;
            break;

        case MAV_CMD_GET_HOME_POSITION:
            break;
        
        default:
            printf("Unknown command long %d\n", cmd.command);
            break;
    }
}

static void mav_send_battery_status() {
    /*
    int16_t voltages[10];
    memset(voltages, 0xff, 20);
    voltages[0] = (int16_t) vbat.scaledMv;
    mavlink_msg_battery_status_pack_chan(
        MAVLINK_SYSTEM_ID,
        MAV_COMP_ID_ALL,
        MAVLINK_CHANNEL_SERIAL,
        &msg_tx,
        0,
        MAV_BATTERY_FUNCTION_AVIONICS,
        MAV_BATTERY_TYPE_LIPO,
        0xFFFF,  // Unknown temp
        voltages,
        -1,
        -1,
        -1,
        -1
    );
    send_mavlink_msg(&msg_tx);*/
}

static void mav_send_raw_imu() {
    /*
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
    */
}

static void mav_send_attitude() {
    /*
    mavlink_msg_attitude_pack_chan(
        MAVLINK_SYSTEM_ID,
        MAV_COMP_ID_IMU,
        MAVLINK_CHANNEL_SERIAL,
        &msg_tx,
        hal_millis(),
        state.roll,
        state.pitch,
        state.yaw,
        state.roll_speed,
        state.pitch_speed,
        state.yaw_speed
    );

    send_mavlink_msg(&msg_tx);*/
}

static void mav_send_rc_channels() {
    /*
    mavlink_msg_rc_channels_pack_chan(
        MAVLINK_SYSTEM_ID,
        0,
        MAVLINK_CHANNEL_SERIAL,
        &msg_tx,
        hal_millis(),
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
    send_mavlink_msg(&msg_tx);*/
}

static void mavlink_driver_statustext(const MAV_SEVERITY severity, const char* text) {
    char buf[50];
    strncpy(buf, text, 50);
    //mavlink_msg_statustext_pack_chan(
    //    MAVLINK_SYSTEM_ID,
    //    0,
    //    MAVLINK_CHANNEL_SERIAL,
    //    &msg_tx,
    //    severity,
    //    buf
    //);
}

static void send_param_value(const char* param_id, const float param_value, const uint16_t param_index)
{
    mavlink_message_t msg;
    char param_id_buf[17];
    memset(param_id_buf, 0, sizeof(param_id_buf));
    strncpy(param_id_buf, param_id, min(sizeof(param_id_buf), 16));

    mavlink_msg_param_value_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &msg,
        param_id_buf,
        param_value,
        MAV_PARAM_TYPE_INT32,
        nbr_of_parameters,
        param_index
    );
    mav_send(&gcs_handler, &msg);
    //printf("Send param %s = %.3f, %d/%d\n", param_id_buf, param_value, param_index+1, nbr_of_parameters);
}

static void send_parameter_request_list()
{
    if (param_index >= nbr_of_parameters)
    {
        send_param_request = false;
        param_index = 0;
        return;
    }
    
    
    send_param_value(get_param_id(param_index), get_param_value(param_index), param_index);
    param_index++;
}

static void handle_param_set(mavlink_message_t* msg)
{
    mavlink_param_set_t param_set;
    mavlink_msg_param_set_decode(msg, &param_set);

    if (set_param_value(param_set.param_id, param_set.param_value, param_set.param_type))
    {
        send_param_value(param_set.param_id, param_set.param_value, 0);
        write_parameters();
    }
}

static void handle_file_transfer_protocol(mavlink_message_t* msg) {
    mavlink_file_transfer_protocol_t ftp;
    mavlink_msg_file_transfer_protocol_decode(msg, &ftp);

    // Prepare a NACK reply
    mavlink_message_t resp;
    mavlink_file_transfer_protocol_t ftp_resp = {0};
    ftp_resp.target_network = ftp.target_network;
    ftp_resp.target_system  = ftp.target_system;
    ftp_resp.target_component = ftp.target_component;

    // Fill in payload as a NACK
    uint8_t* payload = ftp_resp.payload;
    payload[0] = 0x01; // NACK
    payload[1] = ftp.payload[0]; // req_opcode (echo original)
    payload[2] = 0; // result code = "unsupported"

    mavlink_msg_file_transfer_protocol_encode(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &resp,
        &ftp_resp
    );
    mav_send(&gcs_handler, &resp);
}


static void handle_mavlink_message(mavlink_message_t* msg, mavlink_status_t* status)
{
    switch (msg->msgid)
    {
        case MAVLINK_MSG_ID_COMMAND_INT:
            handle_command_int(msg);
            break;

        case MAVLINK_MSG_ID_COMMAND_LONG:
            handle_command_long(msg);
            break;

        case MAVLINK_MSG_ID_HEARTBEAT:
            // No action
            break;

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            send_param_request = true;
            break;

        case MAVLINK_MSG_ID_PARAM_SET:
            handle_param_set(msg);
            break;

        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
            // TODO
            break;

        case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
            handle_file_transfer_protocol(msg);
            break;

        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            handle_rc_channels_override(msg);
            break;
            
        default:
            printf("Unknown message: %d\n", msg->msgid);
            break;
    }
}

#if HAL_SITL
const char* mav_severity_to_str(MAV_SEVERITY sev) {
    switch (sev) {
    case MAV_SEVERITY_EMERGENCY: return "EMERGENCY";
    case MAV_SEVERITY_ALERT:     return "ALERT";
    case MAV_SEVERITY_CRITICAL:  return "CRITICAL";
    case MAV_SEVERITY_ERROR:     return "ERROR";
    case MAV_SEVERITY_WARNING:   return "WARNING";
    case MAV_SEVERITY_NOTICE:    return "NOTICE";
    case MAV_SEVERITY_INFO:      return "INFO";
    case MAV_SEVERITY_DEBUG:     return "DEBUG";
    default:                     return "UNKNOWN";
    }
}
#endif

void gcs_vprintf(const uint8_t severity, const char* fmt, va_list args) {
    char string_buf[256];
    int len = vsnprintf(string_buf, sizeof(string_buf), fmt, args);
    if (len > 0) {
        // assuming serial0 is global or passed in
        mavlink_message_t msg;
        mavlink_msg_statustext_pack(
            MAVLINK_SYSTEM_ID,
            MAVLINK_COMPONENT_ID,
            &msg,
            severity,
             string_buf,
            0,
            0
        );
        mav_send(&gcs_handler, &msg);
    }

    #if HAL_SITL
        printf("[%s] %s\n", mav_severity_to_str(severity), string_buf);
    #endif
}

void gcs_printf(const uint8_t severity, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    gcs_vprintf(severity, fmt, args);
    va_end(args);
}


void mavlink_driver_update()
{
    //if (!usb_connected())
    //{
    //    // USB is not connected, so we don't care about wasting computation resources
    //    // on any type of checks here.
    //    return;
    //}

    // Check RX data from buffer
    int bytes_to_read = min(hal_serial_available(gcs_handler.serial), MAVLINK_INTENRAL_BUF_SIZE);

    // Parse RX data from buffer and handle message
    mavlink_message_t msg;
    mavlink_status_t status;

    uint8_t buf[MAVLINK_INTENRAL_BUF_SIZE];
    int bytes_read = hal_serial_read(gcs_handler.serial, buf, bytes_to_read);

    for (int i = 0; i < bytes_read; i++)
    {
        uint8_t byte = buf[i];

        if (mavlink_parse_char(gcs_handler.channel, byte, &msg, &status))
        {
            handle_mavlink_message(&msg, &status);
        }
    }

    if (send_param_request)
    {
        send_parameter_request_list();
    }

    uint32_t t0 = hal_millis();

    // Check if it's time for any periodic messages to be sent
    if ((t0 - last_sent_heartbeat) >= HEARTBEAT_MSG_PERIOD_MS)
    {
        mav_send_heartbeat(&gcs_handler);
        last_sent_heartbeat = t0;
    }
    if ((t0 - last_sent_battery_status) >= BATTERY_STATUS_PERIOD_MS)
    {
        mav_send_battery_status();
        last_sent_battery_status = t0;
    }
    if ((t0 - last_sent_attitude) >= ATTITUDE_MSG_PERIOD_MS)
    {
        mav_send_attitude();
        last_sent_attitude = t0;
    }
    if ((t0 - last_sent_rc_channels) >= RC_CHANNEL_MSG_PERIOD_MS)
    {
        mav_send_rc_channels();
        last_sent_rc_channels = t0;
    }

    t0 = hal_micros();
}

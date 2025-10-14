#include "mavlink_driver.h"
#include "hal.h"
#include "imu.h"
#include "mavlink_types.h"
#include "scheduler.h"
#include "serial.h"
#include "state.h"
#include "controller.h"
#include "util.h"
#include "param.h"
#include "rc.h"

#include <stdarg.h> // For printf
#include <stdio.h>

#include "mavlink.h"


typedef void (*mavlink_message_transmission_fn)(mavlink_channel_handler_t* channel);

typedef struct
{
    mavlink_message_transmission_fn send;
    uint16_t period_ms;
    uint32_t last_sent;
} message_interval_t;


#define MAVLINK_INTENRAL_BUF_SIZE 10

mavlink_channel_handler_t gcs_handler;
motor_output_t motor_command_test;
static bool send_param_request = false;
static uint32_t param_index = 0;

// Subscribers
on_rc_channels_override_fn on_rc_channels_override_handler = NULL;


// Helper functions
static void handle_mavlink_message(mavlink_message_t* msg, mavlink_status_t* status);


bool mavlink_subscribe_to_rc_channels_override(on_rc_channels_override_fn fn)
{
    if (on_rc_channels_override_handler != NULL) return false;
    on_rc_channels_override_handler = fn;
    return true;
}

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
    if (on_rc_channels_override_handler != NULL)
    {
        on_rc_channels_override_handler(&rc_channels);
    }
}

// Helpers

uint32_t discarded_tx_packets = 0;

void mav_send(mavlink_channel_handler_t* channel, const mavlink_message_t* msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    int bytes_written = hal_serial_write(channel->serial, buf, len);
    if (bytes_written != (int) len)
    {
        discarded_tx_packets++;
    }
}

void send_heartbeat(mavlink_channel_handler_t* channel)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | ((state.armed) ? MAV_MODE_FLAG_SAFETY_ARMED : 0),
    state.flightmode,
MAV_STATE_ACTIVE
);
    mav_send(channel, &msg);
}

static void scan_i2c_bus(const uint8_t bus)
{
    if (bus > 2)
    {
        gcs_printf(MAV_SEVERITY_ERROR, "Invalid i2c bus %d\n", bus);
        return;
    }

    uint8_t devices[20];
    uint8_t devices_found = 0;
    gcs_printf(MAV_SEVERITY_INFO, "Probing i2c bus %d", bus);
    hal_i2c_probe_bus(bus, devices, &devices_found);
    if (devices_found == 0)
    {
        gcs_printf(MAV_SEVERITY_INFO, "No devices found");   
    }
    else
    {
        for (int i = 0; i < devices_found; i++)
        {
            gcs_printf(MAV_SEVERITY_INFO, "[%d] Found device: 0x%02x (%d)", i+1, devices[i], devices[i]);   
        }
    }
}


static void handle_command_int(mavlink_message_t* msg)
{
    mavlink_command_int_t cmd;
    mavlink_msg_command_int_decode(msg, &cmd);
    switch (cmd.command)
    {
        case MAV_CMD_USER_1:
            scan_i2c_bus(cmd.param1);
            break;
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
        default:
            gcs_printf(MAV_SEVERITY_WARNING, "Unsupported command: %d\n", cmd.command);
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
        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (cmd.param1 == 1 && allow_rebooting())
            {
                hal_reboot();
            }
            break;
        case MAV_CMD_COMPONENT_ARM_DISARM:
            state.force_armed = cmd.param1 == 1;
            break;

        case MAV_CMD_GET_HOME_POSITION:
            break;
        
        default:
            printf("Unknown command long %d\n", cmd.command);
            break;
    }
}

static void send_battery_status() {
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

static void send_raw_imu(mavlink_channel_handler_t* channel) {
    mavlink_message_t msg;

    mavlink_msg_scaled_imu_pack_chan(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        MAVLINK_CHANNEL_SERIAL,
        &msg,
        imu_raw.timestamp_us,
        (int16_t) (imu_raw.acc_x * 1000),
        (int16_t) (imu_raw.acc_y * 1000),
        (int16_t) (imu_raw.acc_z * 1000),
        (int16_t) (imu_raw.gyro_x * 1000),
        (int16_t) (imu_raw.gyro_y * 1000),
        (int16_t) (imu_raw.gyro_z * 1000),
        0, 0, 0,
        imu_raw.temp
    );

    mav_send(channel, &msg);
}

static void send_attitude() {
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

static void send_rc_channels(mavlink_channel_handler_t* channel) {
    mavlink_message_t msg;
    mavlink_msg_rc_channels_pack_chan(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, MAVLINK_CHANNEL_SERIAL, &msg,
        hal_millis(),
        18,
        rc_input_raw.channels[0],
        rc_input_raw.channels[1],
        rc_input_raw.channels[2],
        rc_input_raw.channels[3],
        rc_input_raw.channels[4],
        rc_input_raw.channels[5],
        rc_input_raw.channels[6],
        rc_input_raw.channels[7],
        rc_input_raw.channels[8],
        rc_input_raw.channels[9],
        rc_input_raw.channels[10],
        rc_input_raw.channels[11],
        rc_input_raw.channels[12],
        rc_input_raw.channels[13],
        rc_input_raw.channels[14],
        rc_input_raw.channels[15],
        rc_input_raw.channels[16],
        rc_input_raw.channels[17],
        rc_input_raw.rssi
    );
    mav_send(channel, &msg);
}

static void send_rc_channels_scaled(mavlink_channel_handler_t* channel) {
    mavlink_message_t msg;
    mavlink_msg_rc_channels_scaled_pack_chan(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, MAVLINK_CHANNEL_SERIAL, &msg,
        hal_millis(),
        0,
        rc_input_scaled.channels[0],
        rc_input_scaled.channels[1],
        rc_input_scaled.channels[2],
        rc_input_scaled.channels[3],
        rc_input_scaled.channels[4],
        rc_input_scaled.channels[5],
        rc_input_scaled.channels[6],
        rc_input_scaled.channels[7],
        rc_input_scaled.rssi
    );
    mav_send(channel, &msg);
}

static void send_sys_status(mavlink_channel_handler_t* channel)
{
    mavlink_message_t msg;

    uint32_t present_mask = 0;
    uint32_t enabled_mask = 0;
    uint32_t healthy_mask = 0;
    if (imu_sensor.present) present_mask |= (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    if (rc_sensor.present)  present_mask |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;

    if (imu_sensor.enabled) enabled_mask |= (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    if (rc_sensor.enabled)  enabled_mask |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;

    if (imu_sensor.healthy) healthy_mask |= (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    if (rc_sensor.healthy)  healthy_mask |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;


    mavlink_msg_sys_status_pack_chan(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, channel->channel, &msg,
        present_mask,
        enabled_mask,
        healthy_mask,
        scheduler_cpu_load_avg() * 1000,            // 0-1000
        state.bat_volt_mv,               // mV
        state.bat_curr_ma * 10,          // cA -1: not sent
        state.bat_remain,              // %  -1: not sent
        0,                                // Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links
        0,                                   // 	Communication errors (UART, I2C, SPI, CAN), dropped packets on all links 
        0, 0, 0, 0,  // Autopilot-specific errors
        0, 0, 0 // Bitmask extended
    );
    mav_send(channel, &msg);
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



message_interval_t message_intervals[] =
{
    {.send = send_heartbeat,          .period_ms = 1000},
    {.send = send_sys_status,         .period_ms = 500},
    {.send = send_battery_status,     .period_ms = 500},
    {.send = send_attitude,           .period_ms = 100},
    {.send = send_raw_imu,            .period_ms = 100},
    {.send = send_rc_channels,        .period_ms = 100},
    {.send = send_rc_channels_scaled, .period_ms = 100}
};

const int nbr_of_msg_intervals = sizeof(message_intervals) / sizeof(message_interval_t);


int mavlink_driver_init()
{
    for (int i = 0; i  < nbr_of_msg_intervals; i++)
    {
        message_intervals[i].last_sent = 0;
    }

    // TODO: Separate this?
    gcs_handler.channel = 0;
    gcs_handler.serial = &hal_serial0;
    return 0;
}


void mavlink_driver_update()
{
    if (!usb_connected())
    {
        // USB is not connected, so we don't care about wasting computation resources
        // on any type of checks here.
        return;
    }

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

    for (int i = 0; i  < nbr_of_msg_intervals; i++)
    {
        message_interval_t* msg_interval = &message_intervals[i];
        if ((t0 - msg_interval->last_sent) > msg_interval->period_ms)
        {
            msg_interval->send(&gcs_handler);
            msg_interval->last_sent = true;
        }
    }

    t0 = hal_micros();
}

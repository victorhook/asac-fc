#include "crsf.h"

#include "hal.h"

#define CRSF_BAUDRATE 420000

typedef enum
{
    PARSE_DEST      = 0,
    PARSE_LEN       = 1,
    PARSE_TYPE      = 2,
    PARSE_PAYLOAD   = 3,
    PARSE_CRC       = 4
} parse_state_t;


/*
    CRFS Protocol details heavily inspired from this post: https://github.com/ExpressLRS/ExpressLRS/wiki/CRSF-Protocol

    CRFS In short:
        Serial protocol, that uses baud rate 420000, 8 data bits, 1 stop, no parity.
        Data is separated into *frames* with the following structure:
        | dest | len | frame_type | payload | crc |
        CRC is an 8-bit crc with poly 0xD5, including frame_type and payload.
*/

#define CRSF_MAX_PAYLOAD_SIZE 60

typedef struct
{
    unsigned ch0  : 11;
    unsigned ch1  : 11;
    unsigned ch2  : 11;
    unsigned ch3  : 11;
    unsigned ch4  : 11;
    unsigned ch5  : 11;
    unsigned ch6  : 11;
    unsigned ch7  : 11;
    unsigned ch8  : 11;
    unsigned ch9  : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
}__attribute__((packed)) crsf_channels_t;

typedef struct
{
    // Uplink is the connection from the ground to the UAV and downlink the opposite direction.
    uint8_t uplink_rssi_ant1;          // Uplink RSSI Ant. 1 ( dBm * -1 )
    uint8_t uplink_rssi_atn2;          // Uplink RSSI Ant. 2 ( dBm * -1 )
    uint8_t uplink_link_quality;       // Uplink Package success rate / Link quality ( % )
    int8_t  uplink_snr;                // Uplink SNR ( dB, or dB*4 for TBS I believe )
    uint8_t diversity_active_antenna;  // Diversity active antenna ( enum ant. 1 = 0, ant. 2 = 1 )
    uint8_t rf_mode;                   // RF Mode ( 500Hz, 250Hz etc, varies based on ELRS Band or TBS )
    uint8_t uplink_tx_power;           // Uplink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW, 50mW )
    uint8_t downlink_rssi;             // Downlink RSSI ( dBm * -1 )
    uint8_t downlink_link_quality;     // Downlink package success rate / Link quality ( % )
    int8_t  downlink_snr;              // Downlink SNR ( dB )
} crsf_link_statistics_t;


typedef enum
{
    // This enum is 100% copied from betaflight :)
    CRSF_FRAMETYPE_GPS                       = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR            = 0x08,
    CRSF_FRAMETYPE_HEARTBEAT                 = 0x0B,
    CRSF_FRAMETYPE_LINK_STATISTICS           = 0x14,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED        = 0x16,
    CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17,
    CRSF_FRAMETYPE_LINK_STATISTICS_RX        = 0x1C,
    CRSF_FRAMETYPE_LINK_STATISTICS_TX        = 0x1D,
    CRSF_FRAMETYPE_ATTITUDE                  = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE               = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING              = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO              = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ           = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE          = 0x2D,
    CRSF_FRAMETYPE_COMMAND                  = 0x32,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ                  = 0x7A,  // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP                 = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE                = 0x7C,  // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    CRSF_FRAMETYPE_DISPLAYPORT_CMD          = 0x7D  // displayport control command
} crsf_frame_type_t;

typedef enum
{
    CRSF_ADDRESS_CRSF_TRANSMITTER  = 0xEE, // Going to the transmitter module,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA, // Going to the handset,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8, // Going to the flight controller,
    CRSF_ADDRESS_CRSF_RECEIVER     = 0xEC, // Going to the receiver (from FC),
} crsf_dest_t;

typedef struct
{
    uint8_t dest;
    uint8_t len;        // Length of bytes that follow, including type, payload, and CRC (PayloadLength+2). Overall packet length is PayloadLength+4 (dest, len, type, crc), or LEN+2 (dest, len)
    uint8_t frame_type;
    uint8_t payload[CRSF_MAX_PAYLOAD_SIZE];
    uint8_t crc;
}__attribute__((packed)) crsf_packet_t;


#define CRSF_HEADER_SIZE 4

static bool inline valid_dest(const uint8_t byte);
static bool inline valid_length(const uint8_t byte);
static bool inline valid_frame_type(const uint8_t byte);
static bool inline valid_crc(const uint8_t byte);
static void inline reset_rx_state_machine();
static uint8_t calculate_crc(const uint8_t* data, const uint8_t len);
static void handle_new_packet(crsf_packet_t* packet, rc_input_t* rc_input);

static crsf_packet_t rx;
static parse_state_t state;
static uint8_t       payload_bytes_received;
static uint8_t       payload_bytes_to_receive;
static uint32_t      parse_errors;


int crsf_init(serial_t* serial)
{
    parse_errors = 0;
    reset_rx_state_machine();
    return 0;
}

bool crsf_parse_byte(const uint8_t byte, rc_input_t* rc_input)
{
    //printf("State: %d, Len: %d, Payload bytes: %d, %02x\n",
    //        state, rx.len, payload_bytes_received, byte);
    bool new_packet = false;

    switch (state)
    {
        case PARSE_DEST:
            if (valid_dest(byte))
            {
                rx.frame_type = byte;
                state = PARSE_LEN;
            }
            else
            {
                parse_errors++;
            }
            break;
        case PARSE_LEN:
            if (valid_length(byte))
            {
                // Length of the CRSF packet includes {Type, Payload, CRC}
                rx.len = byte;
                payload_bytes_to_receive = rx.len - 2;
                state = PARSE_TYPE;
            }
            else
            {
                parse_errors++;
                reset_rx_state_machine();
            }
            break;
        case PARSE_TYPE:
            if (valid_frame_type(byte))
            {
                rx.frame_type = byte;
                state = PARSE_PAYLOAD;
            }
            else
            {
                reset_rx_state_machine();
                parse_errors++;
            }
            break;
        case PARSE_PAYLOAD:
            rx.payload[payload_bytes_received] = byte;
            payload_bytes_received++;
            if (payload_bytes_received >= payload_bytes_to_receive)
            {
                state = PARSE_CRC;
            }
            break;
        case PARSE_CRC:
            rx.crc = byte;
            if (valid_crc(byte))
            {
                rc_input->timestamp = hal_millis();
                handle_new_packet(&rx, rc_input);
                new_packet = true;
            }
            else
            {
                parse_errors++;
            }
            reset_rx_state_machine();
            break;
        default:
            parse_errors++;
            reset_rx_state_machine();
            break;
    }

    return new_packet;
}

static bool inline valid_dest(const uint8_t byte)
{
    return (
        (byte == CRSF_ADDRESS_CRSF_TRANSMITTER) ||
        (byte == CRSF_ADDRESS_RADIO_TRANSMITTER) ||
        (byte == CRSF_ADDRESS_FLIGHT_CONTROLLER) ||
        (byte == CRSF_ADDRESS_CRSF_RECEIVER)
    );
}
static bool inline valid_length(const uint8_t byte)
{
    // Pretty sure max payload is 64 bytes
    return byte < 64;
}
static bool inline valid_frame_type(const uint8_t byte)
{
    // Highest known frame type (I think?)
    // This is not 100% reliable, but if we get a frame that is unknown
    // we will discard it anyways.
    return byte < 0x7D;
}
static bool inline valid_crc(const uint8_t byte)
{
    uint8_t crc = calculate_crc(&rx.frame_type, rx.len-1);
    return crc == byte;
}
static void inline reset_rx_state_machine()
{
    state = PARSE_DEST;
    payload_bytes_received = 0;
    payload_bytes_to_receive = 0;
}
static uint8_t calculate_crc(const uint8_t* data, const uint8_t len)
{
    // CRSF uses 8-bit CRC with poly 0xD5.
    // Not 100% how this really works but meh :)
    uint8_t poly = 0xD5;
    uint8_t crc = 0;
    for (int i = 0; i < len; i++)
    {
        //printf("CRC: 0x%02x\n", data[i]);
        crc ^= data[i];

        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ poly;
            }
            else
            {
                crc = crc << 1;
            }
        }

    }
    return crc;
}

static void handle_new_packet(crsf_packet_t* packet, rc_input_t* rc_input)
{
    crsf_channels_t* channels;
    crsf_link_statistics_t* statistics;
    switch ((crsf_frame_type_t) packet->frame_type)
    {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            // Fill rx state with channel data
            channels = (crsf_channels_t*) packet->payload;
            rc_input->channels[0]  = channels->ch0;
            rc_input->channels[1]  = channels->ch1;
            rc_input->channels[2]  = channels->ch2;
            rc_input->channels[3]  = channels->ch3;
            rc_input->channels[4]  = channels->ch4;
            rc_input->channels[5]  = channels->ch5;
            rc_input->channels[6]  = channels->ch6;
            rc_input->channels[7]  = channels->ch7;
            rc_input->channels[8]  = channels->ch8;
            rc_input->channels[9]  = channels->ch9;
            rc_input->channels[10] = channels->ch10;
            rc_input->channels[11] = channels->ch11;
            rc_input->channels[12] = channels->ch12;
            rc_input->channels[13] = channels->ch13;
            rc_input->channels[14] = channels->ch14;
            rc_input->channels[15] = channels->ch15;
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS:
            // Fill rx state with link statistics
            statistics = (crsf_link_statistics_t*) packet->payload;
            rc_input->rssi = statistics->uplink_rssi_ant1;
            rc_input->link_quality = statistics->uplink_link_quality;
            break;
        default:
            // TOOD
            printf("UNKNOWN CRSF FRAME TYPE 0x%02x\n", packet->frame_type);
            parse_errors++;
            break;
    }
}

#define CRSF_RC_CHANNEL_SCALE_LEGACY                0.62477120195241f

uint16_t crsf_scale_channel(const uint16_t raw) {
    return (CRSF_RC_CHANNEL_SCALE_LEGACY * (float) raw) + 881;
}


#include "rc/crsf.h"
#include "rc/rc.h"

#include "asac_fc.h"


typedef enum
{
    PARSE_DEST      = 0,
    PARSE_LEN       = 1,
    PARSE_TYPE      = 2,
    PARSE_PAYLOAD   = 3,
    PARSE_CRC       = 4
} parse_state_t;

#define CRSF_HEADER_SIZE 4

static bool inline valid_dest(const uint8_t byte);
static bool inline valid_length(const uint8_t byte);
static bool inline valid_frame_type(const uint8_t byte);
static bool inline valid_crc(const uint8_t byte);
static void inline reset_rx_state_machine();
static uint8_t calculate_crc(const uint8_t* data, const uint8_t len);
static void handle_new_packet(crsf_packet_t* packet);

static rx_state_t    rx_state;
static crsf_packet_t rx;
static parse_state_t state;
static uint8_t       payload_bytes_received;
static uint8_t       payload_bytes_to_receive;
static uint32_t      parse_errors;


int crsf_init()
{
    parse_errors = 0;
    reset_rx_state_machine();
    return 0;
}

bool crsf_parse_byte(const uint8_t byte)
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
                rx_state.last_packet.timestamp = ms_since_boot();
                handle_new_packet(&rx);
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

void crfs_get_last_state(rx_state_t* state)
{
    memcpy(state, &rx_state, sizeof(rx_state_t));
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

static void handle_new_packet(crsf_packet_t* packet)
{
    crsf_channels_t* channels;
    crsf_link_statistics_t* statistics;
    switch ((crsf_frame_type_t) packet->frame_type)
    {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            // Fill rx state with channel data
            channels = (crsf_channels_t*) packet->payload;
            rx_state.last_packet.channels[0] = channels->ch0;
            rx_state.last_packet.channels[1] = channels->ch1;
            rx_state.last_packet.channels[2] = channels->ch2;
            rx_state.last_packet.channels[3] = channels->ch3;
            rx_state.last_packet.channels[4] = channels->ch4;
            rx_state.last_packet.channels[5] = channels->ch5;
            rx_state.last_packet.channels[6] = channels->ch6;
            rx_state.last_packet.channels[7] = channels->ch7;
            rx_state.last_packet.channels[8] = channels->ch8;
            rx_state.last_packet.channels[9] = channels->ch9;
            rx_state.last_packet.channels[10] = channels->ch10;
            rx_state.last_packet.channels[11] = channels->ch11;
            rx_state.last_packet.channels[12] = channels->ch12;
            rx_state.last_packet.channels[13] = channels->ch13;
            rx_state.last_packet.channels[14] = channels->ch14;
            rx_state.last_packet.channels[15] = channels->ch15;
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS:
            // Fill rx state with link statistics
            statistics = (crsf_link_statistics_t*) packet->payload;
            rx_state.statistics.rssi = statistics->uplink_rssi_ant1;
            rx_state.statistics.link_quality = statistics->uplink_link_quality;
            break;
        default:
            // TOOD
            printf("UNKNOWN CRSF FRAME TYPE 0x%02x\n", packet->frame_type);
            parse_errors++;
            break;
    }
}

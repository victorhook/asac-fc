#ifndef CRSF_H
#define CRSF_H

#include "stdint.h"
#include "stdbool.h"

#include "rc/rc.h"

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

/*
 * Initializes the CRSF state machine.
 * Returns 0 on success.
 */
int crsf_init();

/*
 * Parses a single byte in the CRSF state machine.
 * This method returns true if a new packet has been detected, and
 * false otherwise.
 */
bool crsf_parse_byte(const uint8_t byte);

/*
 * Fills the `state` with the current state of the CRSF receiver.
 */
void crfs_get_last_state(rx_state_t* state);


#endif /* CRSF_H */

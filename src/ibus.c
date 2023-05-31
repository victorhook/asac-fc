#include "ibus.h"
#include "asac_fc.h"


#define IBUS_HEADER_FIRST_BYTE  0x20
#define IBUS_HEADER_SECOND_BYTE 0x40
#define IBUS_HEADER_SIZE        2
#define IBUS_PAYLOAD_SIZE       28
#define IBUS_CHECKSUM_SIZE      2
#define IBUS_PACKET_SIZE        (IBUS_HEADER_SIZE + IBUS_PAYLOAD_SIZE + IBUS_CHECKSUM_SIZE)
#define CHECKSUM_START_VALUE    ((uint16_t) (0xFFFF - IBUS_HEADER_FIRST_BYTE - IBUS_HEADER_SECOND_BYTE))

typedef enum {
    HEADER_FIRST_BYTE,
    HEADER_SECOND_BYTE,
    PAYLOAD,
    CHECKSUM_FIRST_BYTE,
    CHECKSUM_SECOND_BYTE,
} ibus_parse_state_t;

// Parsing variables
static ibus_parse_state_t ibus_state;
static uint8_t            buf[IBUS_PACKET_SIZE];
static uint8_t            bytes_read;
static uint16_t           checksum;
static uint16_t           rx_checksum;
static uint32_t           packet_rate_counter_t0;
static uint16_t           packet_rate_counter;
static uint16_t           packet_rate;
static uint32_t           parse_errors;
static uint32_t           successful_packets;
static ibus_packet_t      last_packet;

static void update_statistics();


void ibus_get_statistics(ibus_statistics_t* statistics)
{
    statistics->successful_packets = successful_packets;
    statistics->parse_errors = parse_errors;
    statistics->packet_rate = packet_rate;
    statistics->last_received_packet = last_packet.timestamp;
}

void ibus_init()
{
    ibus_state             = HEADER_FIRST_BYTE;
    bytes_read             = 0;
    parse_errors           = 0;
    last_packet.timestamp  = 0;
    successful_packets     = 0;
    packet_rate_counter    = 0;
    packet_rate_counter_t0 = ms_since_boot();
}

void ibus_process_byte(uint8_t byte)
{
    buf[bytes_read] = byte;
    bytes_read++;
    ibus_parse_state_t next_state = ibus_state;

    switch (ibus_state)
    {
        case HEADER_FIRST_BYTE:
            if (byte == IBUS_HEADER_FIRST_BYTE)
                {   // First byte of header
                    next_state = HEADER_SECOND_BYTE;
                }
                else
                {
                    parse_errors++;
                    bytes_read = 0;
                    next_state = HEADER_FIRST_BYTE;
                }
            break;
        case HEADER_SECOND_BYTE:
            if (byte == IBUS_HEADER_SECOND_BYTE)
            {   // Second byte of header, get ready to parse data
                checksum = CHECKSUM_START_VALUE;
                next_state = PAYLOAD;
            }
            else
            {
                parse_errors++;
                bytes_read = 0;
                next_state = HEADER_FIRST_BYTE;
            }
            break;
        case PAYLOAD:
            // Update checksum (subtract data value)
            checksum -= byte;
            if (bytes_read == (IBUS_PAYLOAD_SIZE + IBUS_HEADER_SIZE))
            {
                next_state = CHECKSUM_FIRST_BYTE;
            }
            break;
        case CHECKSUM_FIRST_BYTE:
            rx_checksum = byte;
            next_state = CHECKSUM_SECOND_BYTE;
            break;
        case CHECKSUM_SECOND_BYTE:
            rx_checksum |= (byte << 8);

            // Validate checksum
            if ((rx_checksum) != checksum)
            {
                parse_errors++;
                //printf("ERR: %d - %d -> ||", rx_checksum, checksum);
            }
            else
            {
                memcpy(last_packet.channels,
                       &buf[IBUS_HEADER_SIZE],
                       IBUS_PAYLOAD_SIZE);
                last_packet.timestamp = ms_since_boot();
                successful_packets++;
                packet_rate_counter++;
            }

            bytes_read = 0;
            next_state = HEADER_FIRST_BYTE;
            break;
    }

    update_statistics();

    ibus_state = next_state;
}

void ibus_get_last_packet(ibus_packet_t* packet)
{
    memcpy(packet, &last_packet, sizeof(ibus_packet_t));
}


static void update_statistics()
{
    uint32_t now = ms_since_boot();
    if ((now - packet_rate_counter_t0) > 1000 )
    {
        // Update statistics
        packet_rate = packet_rate_counter;
        packet_rate_counter = 0;
        packet_rate_counter_t0 = now;
        //printf("Errs: %d, %d\n", parse_errors, packet_rate);
    }
}
#include "ibus.h"

#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"


#define IBUS_HEADER_FIRST_BYTE  0x20
#define IBUS_HEADER_SECOND_BYTE 0x40
#define IBUS_HEADER_SIZE        2
#define IBUS_PAYLOAD_SIZE       28
#define IBUS_CHECKSUM_SIZE      2
#define IBUS_PACKET_SIZE        (IBUS_HEADER_SIZE + IBUS_PAYLOAD_SIZE + IBUS_CHECKSUM_SIZE)


static ibus_packet_t last_packet;


uint32_t getCurrentTime() {
    return time_us_32();
}

void ibus_process_byte(uint8_t data)
{
    static uint8_t bytes_read = 0;
    static uint8_t buf[IBUS_PAYLOAD_SIZE];
    static uint8_t checksum[2];

    if (bytes_read == 0)
    {
        if (data == IBUS_HEADER_FIRST_BYTE)
        {
            // First byte of header
            bytes_read++;
        }
    } else if (bytes_read == 1)
    {
        if (data == IBUS_HEADER_SECOND_BYTE)
        {
            // Second byte of header
            bytes_read++;
        }
    } else if (bytes_read <= (IBUS_HEADER_SIZE + IBUS_PAYLOAD_SIZE))
    {
        // Channel data
        buf[bytes_read - IBUS_HEADER_SIZE] = data;
        bytes_read++;
    } else if (bytes_read <= IBUS_PACKET_SIZE)
    {
        // Checksum
        checksum[bytes_read - IBUS_HEADER_SIZE - IBUS_PAYLOAD_SIZE] = data;
        bytes_read++;
    } else if (bytes_read > IBUS_PACKET_SIZE)
    {
        // We've finished reading a packet.
        bytes_read = 0;

        // TODO: Check checksum

        memcpy(last_packet.channels, buf, 14 * 2);
        last_packet.timestamp = getCurrentTime();
    }
}

void ibus_get_last_packet(ibus_packet_t* packet)
{
    memcpy(packet, &last_packet, sizeof(ibus_packet_t));
}

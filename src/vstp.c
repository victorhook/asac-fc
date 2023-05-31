#include "vstp.h"


void vstp_init(vstp_client_t* client, uart_write_bytes uart_write)
{
    client->write = uart_write;
}

void vstp_transmit(vstp_client_t* client, const vstp_cmd_t cmd, const uint8_t* data, const uint8_t len)
{
    client->tx.cmd = (uint8_t) cmd;
    client->tx.len = len;
    client->tx.crc = cmd ^ len;
    for (int i = 0; i < len; i++)
    {
        uint8_t byte = data[i];
        client->tx.crc ^= byte;
        client->tx.data[i] = byte;
    }

    // We'll write the vstp packet to client.
    // Note that size of the entire vstp packet (including payload) must be
    // the payload size + header size.
    client->write((uint8_t*) &client->tx, len + VSTP_PACKET_HEADER_SIZE);
}


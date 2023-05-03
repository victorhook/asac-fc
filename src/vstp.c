#include "vstp.h"



void vstp_init(vstp_client_t* client, uart_write_bytes uart_write)
{
    client->write = uart_write;
}

void vstp_transmit(vstp_client_t* client, const vstp_cmd_t cmd, const char* data, const uint16_t len)
{
    client->tx.cmd = cmd;
    client->tx.len = len;
    client->tx.crc = cmd ^ len;
    for (int i = 0; i < len; i++)
    {
        client->tx.crc ^= data[i];
    }

    client->write((uint8_t*) &client->tx, len);
}


#ifndef VSTP_H
#define VSTP_H

#include "stdint.h"
#include "stdbool.h"


#define VSTP_PACKET_HEADER_SIZE      3
#define VSTP_PACKET_MAX_PAYLOAD_SIZE (0xFF - VSTP_PACKET_HEADER_SIZE)


typedef enum {
    VSTP_CMD_LOG_START    = 1,
    VSTP_CMD_LOG_STOP     = 2,
    VSTP_CMD_LOG_DATA     = 3,
    VSTP_CMD_LOG_SD_START = 4,
    VSTP_CMD_LOG_SD_STOP  = 5,
    VSTP_CMD_RESET        = 6
} vstp_cmd_t;

// This is used for validating commands, please update accordingly
#define VSTP_NBR_OF_CMDS 5

typedef struct {
    uint8_t cmd;  // vstp_cmd_t
    uint8_t len;
    uint8_t crc;
    uint8_t data[VSTP_PACKET_MAX_PAYLOAD_SIZE];
}__attribute__((packed)) vstp_pkt_t;

/*
 * Writes len bytes through UART.
 */
typedef void(*uart_write_bytes)(const uint8_t* data, const uint8_t len);

typedef struct {
    uart_write_bytes write;
    vstp_pkt_t tx;
} vstp_client_t;


/*
 * Initializes the vstp client
 */
void vstp_init(vstp_client_t* client, uart_write_bytes uart_write);

/*
 * Transmits a VSTP packet.
 * data: Can be NULL if no data needed
 */
void vstp_transmit(vstp_client_t* client, const vstp_cmd_t cmd, const uint8_t* data, const uint8_t len);



#endif /* VSTP_H */

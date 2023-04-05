#ifndef MSP_H
#define MSP_H

#include "stdint.h"


// MSP INFO
#define INFO_FC_VARIANT "ASAC"
#define INFO_FC_VERSION 0
#define INFO_BOARD_INFO "ASAC"
#define INFO_BOARD_VERSION 0
#define INFO_BUILD_DATE "2023-03-05"
#define INFO_BUILD_TIME "19:03:00"


#define MSP_MAX_DATA_SIZE 128


typedef enum {
    MSP_API_VERSION 	  = 1,
    MSP_FC_VARIANT 	      = 2,
    MSP_FC_VERSION 	      = 3,
    MSP_BOARD_INFO        = 4,
    MSP_BUILD_INFO        = 5,
    MSP_IDENT             = 100,
    MSP_RAW_IMU           = 102,
    MSP_MOTOR             = 104,
    MSP_SET_MOTOR         = 214,
    MSP_ATTITUDE          = 108,
    MSP_PID               = 112,
    MSP_SET_PID           = 202,

    // CUSTOM
    MSP_READ_MEM_ADDRESS  = 180,
    MSP_WRITE_MEM_ADDRESS = 181
} msp_command_t;



// <preamble>,<direction>,<size>,<command>,,<crc>
// http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
typedef struct {
    // preamble = the ASCII characters '$M'
    uint8_t preamble[2];
    // direction = the ASCII character '<' if to the MWC or '>' if from the MWC
    uint8_t direction;
    // size = number of data bytes, binary. Can be zero as in the case of a data request to the MWC
    uint8_t size;
    // command = message_id as per the table below
    msp_command_t command;
    // data = as per the table below. UINT16 values are LSB first.
    uint8_t data[MSP_MAX_DATA_SIZE];
    // crc = XOR of <size>, <command> and each data byte into a zero'ed sum
    uint8_t crc;
} __attribute__((packed)) msp_packet_t;


typedef void (*response_callback)(const uint8_t* data, const uint8_t len);

#define MSP_RX_BUF_LEN 128
#define MSP_TX_BUF_LEN 128

typedef struct {
    response_callback callback;
    msp_packet_t rx_pkt;
    uint8_t tx_buf[256];
    uint8_t tx_len;
} msp_state_t;


int msp_init(msp_state_t* state);

/* Processes a single byte of an MSP packet. */
void msp_process_byte(msp_state_t* state, const uint8_t byte);

/* Sets the callback function to use when responding to MSP packets. */
void msp_set_response_callback(msp_state_t* state, response_callback callback);


#endif /* MSP_H */

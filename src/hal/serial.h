
#ifndef SERIAL_H
#define SERIAL_H

#include "util/ringbuf.h"

#define SERIAL_RX_BUFF_SIZE 1048
#define SERIAL_TX_BUFF_SIZE 1048


typedef struct
{
    uint8_t  rx;
    uint8_t  tx;
    uint32_t baud;
} bus_config_serial_t;

typedef enum
{
    SERIAL_PROTOCOL_DISABLED = 0,
    SERIAL_PROTOCOL_CRSF     = 1,
    SERIAL_PROTOCOL_MAVLINK  = 2,
} serial_protocol_t;

typedef struct
{
    uint8_t             nbr;
    ringbuf_t           rx_buf;
    ringbuf_t           tx_buf;
    bus_config_serial_t config;
    serial_protocol_t   protocol;
} serial_t;


// -- Abstract -- //

// Hal specific update
void hal_serial_do_update();

/** Initializes the serial port with given number and baudrate */
int hal_serial_init(const bus_config_serial_t config, const uint8_t nbr);

int hal_serial_do_write(serial_t* serial, const uint8_t* data, const uint32_t len);

int hal_serial_do_read(serial_t* serial, uint8_t* data, const uint32_t len);


// -- Base -- //

/** Returns number of bytes available in the serial RX buffer */
int hal_serial_available(const serial_t* serial);

/** Writes data to the serial port (non-blocking). Returns the number of bytes written. */
int hal_serial_write(serial_t* serial, const uint8_t* data, const uint32_t len);

/** Reads number of bytes into `data`. Returns the number of bytes read. */
int hal_serial_read(serial_t* serial, uint8_t* data, const uint32_t len);

void hal_serial_update();


#endif
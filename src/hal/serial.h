
#ifndef SERIAL_H
#define SERIAL_H

#include "util/ringbuf.h"

#define SERIAL_RX_BUFF_SIZE 1048
#define SERIAL_TX_BUFF_SIZE 1048

typedef struct
{
    uint8_t nbr;
    ringbuf_t rx_buf;
    ringbuf_t tx_buf;
} serial_t;


/** Initializes the serial port with given number and baudrate */
int hal_serial_init(serial_t* serial, const uint8_t serial_nbr, const uint32_t baudrate);

/** Writes data to the serial port (non-blocking). Returns the number of bytes written. */
int hal_serial_write(serial_t* serial, const uint8_t* data, const uint16_t len);

/** Reads number of bytes into `data`. Returns the number of bytes read. */
int hal_serial_read(serial_t* serial, uint8_t* data, const uint16_t len);

/** Returns number of bytes available in the serial RX buffer */
int hal_serial_available(const serial_t* serial);


#endif
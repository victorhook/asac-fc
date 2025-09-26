
#ifndef SERIAL_H
#define SERIAL_H

#include "util/ringbuf.h"

#define SERIAL_RX_BUFF_SIZE 2048
#define SERIAL_TX_BUFF_SIZE 2048

typedef struct
{
    uint8_t nbr;
    ringbuf_t rx_buf;
    ringbuf_t tx_buf;
} serial_t;

int serial_usb_init();

int serials_hardware_init();

extern serial_t serial0;
extern serial_t serial1;

#endif
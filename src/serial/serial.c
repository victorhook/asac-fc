#include "serial.h"
#include "hal.h"


// SERIAL0
uint8_t   serial0_rx_buf[SERIAL_RX_BUFF_SIZE];
uint8_t   serial0_tx_buf[SERIAL_TX_BUFF_SIZE];
serial_t  serial0;

// SERIAL1
uint8_t   serial1_rx_buf[SERIAL_RX_BUFF_SIZE];
uint8_t   serial1_tx_buf[SERIAL_TX_BUFF_SIZE];
serial_t  serial1;


static int serial_init(serial_t* serial, const uint8_t number, const uint32_t baudrate, uint8_t* rx_buf, uint8_t* tx_buf, const uint16_t rx_buf_size, const uint16_t tx_buf_size);

int serial_usb_init()
{
    serial_init(&serial0, 0, 921600, serial0_rx_buf, serial0_tx_buf, SERIAL_RX_BUFF_SIZE, SERIAL_TX_BUFF_SIZE);
    return 0;
}

int serials_hardware_init()
{
    //serial_init(&serial1, 1, 921600, serial0_rx_buf, serial0_tx_buf, SERIAL_RX_BUFF_SIZE, SERIAL_TX_BUFF_SIZE);
    return 0;
}

static int serial_init(serial_t* serial, const uint8_t number, const uint32_t baudrate, uint8_t* rx_buf, uint8_t* tx_buf, const uint16_t rx_buf_size, const uint16_t tx_buf_size)
{
    serial->nbr = number;
    ringbuf_init(&serial->rx_buf, rx_buf, rx_buf_size);
    ringbuf_init(&serial->tx_buf, tx_buf, tx_buf_size);
    return hal_serial_init(&serial0, 0, baudrate) == 0;
}

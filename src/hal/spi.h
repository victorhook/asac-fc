
#ifndef SPI_H
#define SPI_H

#include "util/ringbuf.h"

#define SPI_RX_BUFF_SIZE 256
#define SPI_TX_BUFF_SIZE 256


typedef struct
{
    uint8_t mosi;
    uint8_t miso;
    uint8_t clk;
    uint8_t ss;
    uint32_t freq;
} bus_config_spi_t;

typedef struct
{
    uint8_t          nbr;
    ringbuf_t        rx_buf;
    ringbuf_t        tx_buf;
    bus_config_spi_t config;
} spi_t;

int hal_spi_init(const bus_config_spi_t config);

int hal_spi_read(spi_t* spi, const uint8_t addr, uint8_t *data, const uint32_t len);

int hal_spi_read_byte(spi_t* spi, const uint8_t addr, uint8_t *byte);

int hal_spi_write(spi_t* spi, const uint8_t addr, const uint8_t *data, const uint32_t len);

int hal_spi_write_byte(spi_t* spi, const uint8_t addr, const uint8_t *data, const uint32_t len);

#endif
#include "hal.h"
#include "i2c.h"
#include "ringbuf.h"
#include "serial.h"
#include "spi.h"

// I2C 2
extern float brd_i2c1_sda;
extern float brd_i2c1_scl;
extern float brd_i2c1_freq;

// I2C 1
extern float brd_i2c2_sda;
extern float brd_i2c2_scl;
extern float brd_i2c2_freq;

// SPI 1
extern float brd_spi1_mosi;
extern float brd_spi1_miso;
extern float brd_spi1_clk;
extern float brd_spi1_freq;

// SPI 2
extern float brd_spi2_mosi;
extern float brd_spi2_miso;
extern float brd_spi2_clk;
extern float brd_spi2_freq;

// LEDs
extern float brd_led1;
extern float brd_led2;
extern float brd_led3;

// IMU
extern float brd_imu_type;
extern float brd_imu_bus;
extern float brd_imu_ss;

// Serial 1
extern float brd_serial1_rx;
extern float brd_serial1_tx;
extern float brd_serial1_baud;
extern float brd_serial1_protocol;

// Serial 2
extern float brd_serial2_rx;
extern float brd_serial2_tx;
extern float brd_serial2_baud;
extern float brd_serial2_protocol;


// -- Instantiate communication buses -- //

// SERIAL0
uint8_t   serial0_rx_buf[SERIAL_RX_BUFF_SIZE];
uint8_t   serial0_tx_buf[SERIAL_TX_BUFF_SIZE];
serial_t  serial0;

// SERIAL1
uint8_t   serial1_rx_buf[SERIAL_RX_BUFF_SIZE];
uint8_t   serial1_tx_buf[SERIAL_TX_BUFF_SIZE];
serial_t  serial1;

uint8_t   serial2_rx_buf[SERIAL_RX_BUFF_SIZE];
uint8_t   serial2_tx_buf[SERIAL_TX_BUFF_SIZE];
serial_t  serial2;

// I2C1
uint8_t i2c1_rx_buf[I2C_RX_BUFF_SIZE];
uint8_t i2c1_tx_buf[I2C_TX_BUFF_SIZE];
i2c_t   i2c1;

// I2C2
uint8_t i2c2_rx_buf[I2C_RX_BUFF_SIZE];
uint8_t i2c2_tx_buf[I2C_TX_BUFF_SIZE];
i2c_t   i2c2;

// SPI1
uint8_t spi1_rx_buf[SPI_RX_BUFF_SIZE];
uint8_t spi1_tx_buf[SPI_TX_BUFF_SIZE];
spi_t   spi1;

// SPI2
uint8_t spi2_rx_buf[SPI_RX_BUFF_SIZE];
uint8_t spi2_tx_buf[SPI_TX_BUFF_SIZE];
spi_t   spi2;


serial_t* serials[] = {&serial0, &serial1, &serial2};
spi_t* spis[] = {&spi1, &spi2};
i2c_t* i2cs[] = {&i2c1, &i2c2};

const int nbr_of_serials = sizeof(serials) / sizeof(serial_t*);
const int nbr_of_spi = sizeof(spis) / sizeof(spi_t*);
const int nbr_of_i2c = sizeof(i2cs) / sizeof(i2c_t*);


static int bus_serial_init(serial_t* serial, const uint8_t number, const int rx, const int tx, const uint32_t baudrate, uint8_t* rx_buf, uint8_t* tx_buf, const uint16_t rx_buf_size, const uint16_t tx_buf_size)
{
    if ((rx < 0) || (tx < 0)) return -1;

    bus_config_serial_t config =
    {
        .rx = rx,
        .tx = tx,
        .baud = baudrate
    };
    serial->nbr = number;
    ringbuf_init(&serial->rx_buf, rx_buf, rx_buf_size);
    ringbuf_init(&serial->tx_buf, tx_buf, tx_buf_size);
    return hal_serial_init(config, number);
}

static int bus_i2c_init(const uint8_t bus, const uint8_t sda, const uint8_t scl, const uint32_t freq, uint8_t* rx_buf, uint8_t* tx_buf, const uint16_t rx_buf_size, const uint16_t tx_buf_size)
{
    bus_config_i2c_t config =
    {
        .sda = sda,
        .scl = scl,
        .freq = freq
    };
    return hal_i2c_init(config);
}

static int bus_spi_init(const uint8_t bus, const uint8_t mosi, const uint8_t miso, const uint8_t clk, const uint32_t freq, uint8_t* rx_buf, uint8_t* tx_buf, const uint16_t rx_buf_size, const uint16_t tx_buf_size)
{
    bus_config_spi_t config =
    {
        .mosi = mosi,
        .miso = miso,
        .clk = clk,
        .freq =freq
    };
    return hal_spi_init(config);
}

int hal_init()
{
    int res = 0;
    res |= bus_serial_init(&serial0, 0, 921600, 0, 0, serial0_rx_buf, serial0_tx_buf, SERIAL_RX_BUFF_SIZE, SERIAL_TX_BUFF_SIZE);
    res |= bus_serial_init(&serial1, 1, brd_serial1_rx, brd_serial1_tx, brd_serial1_baud, serial1_rx_buf, serial1_tx_buf, SERIAL_RX_BUFF_SIZE, SERIAL_TX_BUFF_SIZE);
    res |= bus_serial_init(&serial2, 2, brd_serial2_rx, brd_serial2_tx, brd_serial2_baud, serial2_rx_buf, serial2_tx_buf, SERIAL_RX_BUFF_SIZE, SERIAL_TX_BUFF_SIZE);

    res |= bus_i2c_init(1, brd_i2c1_sda, brd_i2c1_scl, brd_i2c1_freq, i2c1_rx_buf, i2c1_tx_buf, I2C_RX_BUFF_SIZE, I2C_TX_BUFF_SIZE);
    res |= bus_i2c_init(2, brd_i2c2_sda, brd_i2c2_scl, brd_i2c2_freq, i2c2_rx_buf, i2c2_tx_buf, I2C_RX_BUFF_SIZE, I2C_TX_BUFF_SIZE);

    res |= bus_spi_init(1, brd_spi1_mosi, brd_spi1_miso, brd_spi1_clk, brd_spi1_freq, spi1_rx_buf, spi1_tx_buf, SPI_RX_BUFF_SIZE, SPI_TX_BUFF_SIZE);
    res |= bus_spi_init(2, brd_spi2_mosi, brd_spi2_miso, brd_spi2_clk, brd_spi2_freq, spi2_rx_buf, spi2_tx_buf, SPI_RX_BUFF_SIZE, SPI_TX_BUFF_SIZE);

    res |= hal_gpio_init(brd_led1, HAL_GPIO_FUNCTION_OUTPUT, HAL_GPIO_LOW);
    res |= hal_gpio_init(brd_led2, HAL_GPIO_FUNCTION_OUTPUT, HAL_GPIO_LOW);
    res |= hal_gpio_init(brd_led3, HAL_GPIO_FUNCTION_OUTPUT, HAL_GPIO_LOW);

    res |=  hal_do_init();
    return res;
}


static uint32_t do_write(void* serial, const uint8_t* data, const uint32_t len)
{
    hal_serial_do_write(((serial_t*) serial), data, len);
}

void hal_serial_update()
{
    for (int i = 0; i < nbr_of_serials; i++)
    {
        serial_t* serial = serials[i];

        // Call hal-specific updates (if needed)
        hal_serial_do_update(serial);

        // Read from TX buffer into HAL
        ringbuf_consume(&serial->tx_buf, (ringbuf_consumer_fn) hal_serial_do_write, serial, 0);

        // Write from HAL into RX buffer
        ringbuf_produce(&serial->rx_buf, (ringbuf_producer_fn) hal_serial_do_read, serial, 0);
    }
}

int hal_serial_write(serial_t* serial, const uint8_t* data, const uint32_t len)
{
    return ringbuf_write(&serial->tx_buf, data, len) ? len : 0;
}

int hal_serial_read(serial_t* serial, uint8_t* data, const uint32_t len)
{
    return ringbuf_read(&serial->rx_buf, data, len) ? len : 0;
}

int hal_serial_available(const serial_t* serial)
{
    return ringbuf_items(&serial->rx_buf);
}

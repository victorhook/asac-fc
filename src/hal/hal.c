#include "hal.h"
#include "i2c.h"
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

// IMU
extern float brd_imu_type;
extern float brd_imu_bus;
extern float brd_imu_ss;

// Instantiate buses


// SERIAL0
uint8_t   serial0_rx_buf[SERIAL_RX_BUFF_SIZE];
uint8_t   serial0_tx_buf[SERIAL_TX_BUFF_SIZE];
serial_t  serial0;

// SERIAL1
uint8_t   serial1_rx_buf[SERIAL_RX_BUFF_SIZE];
uint8_t   serial1_tx_buf[SERIAL_TX_BUFF_SIZE];
serial_t  serial1;

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


static int bus_serial_init(serial_t* serial, const uint8_t number, const uint32_t baudrate, uint8_t* rx_buf, uint8_t* tx_buf, const uint16_t rx_buf_size, const uint16_t tx_buf_size)
{
    serial->nbr = number;
    ringbuf_init(&serial->rx_buf, rx_buf, rx_buf_size);
    ringbuf_init(&serial->tx_buf, tx_buf, tx_buf_size);
    return hal_serial_init(&serial0, 0, baudrate) == 0;
}

static int bus_i2c_init(const uint8_t bus, const uint8_t sda, const uint8_t scl, const uint32_t freq, uint8_t* rx_buf, uint8_t* tx_buf, const uint16_t rx_buf_size, const uint16_t tx_buf_size)
{
    bus_config_i2c_t config =
    {
        .nbr = bus,
        .sda = sda,
        .scl = scl,
        .freq =freq
    };
    return hal_i2c_init(config);
}

static int bus_spi_init(const uint8_t bus, const uint8_t mosi, const uint8_t miso, const uint8_t clk, const uint32_t freq, uint8_t* rx_buf, uint8_t* tx_buf, const uint16_t rx_buf_size, const uint16_t tx_buf_size)
{
    bus_config_spi_t config =
    {
        .nbr = bus,
        .mosi = mosi,
        .miso = miso,
        .clk = clk,
        .freq =freq
    };
    return hal_spi_init(config);
}

int hal_init()
{
    bus_serial_init(&serial0, 0, 921600, serial0_rx_buf, serial0_tx_buf, SERIAL_RX_BUFF_SIZE, SERIAL_TX_BUFF_SIZE);

    /*
    bus_i2c_init(1, brd_i2c1_sda, brd_i2c1_scl, brd_i2c1_freq);
    bus_i2c_init( 2, brd_i2c2_sda, brd_i2c2_scl, brd_i2c2_freq);

    bus_spi_init(1, brd_spi1_mosi, brd_spi1_miso, brd_spi1_clk, brd_spi1_freq);
    bus_spi_init(2, brd_spi2_mosi, brd_spi2_miso, brd_spi2_clk, brd_spi2_freq);

    hal_gpio_init(brd_led1, HAL_GPIO_FUNCTION_OUTPUT, HAL_GPIO_LOW);
    hal_gpio_init(brd_led2, HAL_GPIO_FUNCTION_OUTPUT, HAL_GPIO_LOW);

    printf("HAL INIT OK :)\n");
    */

    return hal_do_init();
}



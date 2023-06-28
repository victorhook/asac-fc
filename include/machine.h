#ifndef MACHINE_H
#define MACHINE_H


// Pin definitions for custom PCB - Revision B
// TODO: Incorrect motor names!!
// M1 in schematic and PCB is actually M2 according to motor mixer :)
// So M1 and M2 are swapped
//#define PIN_M1                 1
//#define PIN_M2                 0
#define PIN_M3                20
#define PIN_M4                18
#define PIN_M1                 0
#define PIN_M2                 1

#define PIN_VBAT_ADC          28
#define VBAT_ADC_INPUT_NUMBER  2

#define PIN_IMU_MOSI          11
#define PIN_IMU_MISO          12
#define PIN_IMU_CS            13
#define PIN_IMU_SCK           14
#define PIN_IMU_INTERRUPT_1   10
#define PIN_IMU_INTERRUPT_2    9
#define IMU_SPI_BUS           spi1

#define PIN_SDA1               2
#define PIN_SCL1               3

#define PIN_TX1                4
#define PIN_RX1                5
#define PIN_TX2_PIO            6
#define PIN_RX2_PIO            7

#define PIN_LED_BLUE           8
#define PIN_LED_GREEN          27
#define PIN_LED_RED            26

#define PIN_TELEMETRY_TX 0

#define ASAC_FC_FLASH_SIZE        (2 * 1024 * 1024)

/*
// Pin definitions for dev-and breakout boards.
#define PIN_M1          13 // 6B
#define PIN_M2          19 // 1B
#define PIN_M3          2  // 1A
#define PIN_M4          27 // 5B
#define PIN_M_DEBUG     16 // 0A

#define PIN_VUSB_SENSE  24
#define PIN_LED_ONBOARD 25

#define PIN_LED_RED     10
#define PIN_LED_GREEN   5

#define PIN_RX1         9
#define PIN_SCL1        7
#define PIN_SDA1        6

#define PIN_IBUS_RX PIN_RX1
#define IBUS_UART uart1

#define I2C_BUS_IMU i2c1

#define PIN_TELEMETRY_TX 16

#define VBAT_ADC_INPUT_NUMBER 2
#define PIN_VBAT_ADC          28
*/

#endif

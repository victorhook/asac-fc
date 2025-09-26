#ifndef MACHINE_H
#define MACHINE_H


// Pin definitions for custom PCB - Revision B
// TODO: Incorrect motor names!!
// Motor MIXER M1 and M2 are swapped. The motor mixer uses betaflights
// motor mix!
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

#define PIN_VUSB_SENSE        19

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

// 16 MB
#define ASAC_FC_FLASH_SIZE        (16 * 1024 * 1024)


#endif

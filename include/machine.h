#ifndef MACHINE_H
#define MACHINE_H

/*
Pin definitions for custom PCB
#define PIN_M1            3
#define PIN_M2            0
#define PIN_M3            25
#define PIN_M4            17

#define PIN_BUZZER        11
#define PIN_CURR          1
#define PIN_VBAT_ADC      27
#define VBAT_ADC_INPUT_NUMBER   1
#define PIN_IMU_INTERRUPT 2
#define PIN_RX_ESC        29

#define IMU_I2C_NUMBER 0
#define PIN_SDA0          4
#define PIN_SCL0          5
#define PIN_SDA1          6
#define PIN_SCL1          7

#define PIN_TX1           8
#define PIN_RX1           9
#define PIN_TX0           12
#define PIN_RX0           13

#define PIN_LED           10
#define PIN_LED_BLUE      18
#define PIN_LED_GREEN     19
#define PIN_LED_RED       26
*/

#define ASAC_FC_FLASH_SIZE        (2 * 1024 * 1024)

// Pin definitions for dev-and breakout boards.
#define PIN_M1          13
#define PIN_M2          18
#define PIN_M3          2
#define PIN_M4          27

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

#define VBAT_ADC_INPUT_NUMBER 2
#define PIN_VBAT_ADC          28


#endif

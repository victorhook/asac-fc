#include "hal.h"
#include "vendor/pico-sdk/lib/tinyusb/src/class/cdc/cdc_device.h"
#include "vendor/pico-sdk/lib/tinyusb/src/device/usbd.h"



int hal_serial_init(serial_t* serial, const uint8_t serial_nbr, const uint32_t baudrate)
{
    return 0;
}

int hal_serial_write(serial_t* serial, const uint8_t* data, const uint16_t len)
{
    return tud_cdc_write(data, len);
}

int hal_serial_read(serial_t* serial, uint8_t* data, const uint16_t len)
{
    tud_task();
    return tud_cdc_read(data, len);
}

int hal_serial_available(const uint8_t serial_nbr)
{
    return tud_cdc_available();
}
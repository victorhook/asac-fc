#include "hal.h"
#include "vendor/pico-sdk/lib/tinyusb/src/class/cdc/cdc_device.h"
#include "vendor/pico-sdk/lib/tinyusb/src/device/usbd.h"

#include <pico/stdio.h>


static inline int hal_usb_serial_init()
{
    stdio_usb_init();
}

static inline int hal_usb_serial_write(const uint8_t* data, const uint16_t len)
{
    int written = tud_cdc_write(data, len);
    tud_cdc_n_write_flush(0);
    return written;
}

static inline int hal_usb_serial_read(uint8_t* data, const uint16_t len)
{
    return tud_cdc_read(data, len);
}

static inline int hal_usb_serial_available()
{
    // Must call tud task to poll the USB controller
    tud_task();
    return tud_cdc_available();
}


int hal_serial_init(serial_t* serial, const uint8_t serial_nbr, const uint32_t baudrate)
{
    if (serial_nbr == 0)
    {
        return hal_usb_serial_init();
    }
    return 0;
}

int hal_serial_write(serial_t* serial, const uint8_t* data, const uint16_t len)
{
    if (serial->nbr == 0)
    {
        return hal_usb_serial_write(data, len);
    }
    return 0;
}

int hal_serial_read(serial_t* serial, uint8_t* data, const uint16_t len)
{
    if (serial->nbr == 0)
    {
        return hal_usb_serial_read(data, len);
    }
    return 0;
}

int hal_serial_available(const serial_t* serial)
{
    if (serial->nbr == 0)
    {
        return hal_usb_serial_available();
    }
    return 0;
}

#include "hal.h"
#include "ringbuf.h"
#include "tusb.h"

#include <pico/stdio.h>


static inline int hal_usb_serial_init()
{
    stdio_usb_init();
}

static inline int hal_usb_serial_write(const uint8_t* data, const uint32_t len)
{
    int written = tud_cdc_write(data, len);
    tud_cdc_n_write_flush(0);
    return written;
}

static inline int hal_usb_serial_read(uint8_t* data, const uint32_t len)
{
    return tud_cdc_read(data, len);
}


int hal_serial_init(const bus_config_serial_t config, const uint8_t nbr)
{
    if (nbr == 0)
    {
        return hal_usb_serial_init();
    }
    return 0;
}

int hal_serial_do_write(serial_t* serial, const uint8_t* data, const uint32_t len)
{
    if (serial->nbr == 0 && usb_connected())
    {
        return hal_usb_serial_write(data, len);
    }
}

int hal_serial_do_read(serial_t* serial, uint8_t* data, const uint32_t len)
{
    if (serial->nbr == 0 && usb_connected())
    {
        return hal_usb_serial_read(data, len);
    }
    return 0;
}

bool usb_connected()
{
    return tud_cdc_connected();
}

void hal_serial_do_update()
{
    // Must call tud task to poll the USB controller
    tud_task();
}
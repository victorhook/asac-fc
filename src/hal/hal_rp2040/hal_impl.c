#include "hal_impl.h"


void system_init()
{
    // Initialize misc system stuff that isn't covered by any specific driver

    // Pin to sense if USB is connected or not
    gpio_init(PIN_VUSB_SENSE);
    gpio_set_dir(PIN_VUSB_SENSE, GPIO_IN);
}

bool usb_connected()
{
    return gpio_get(PIN_VUSB_SENSE) != 0;
}

void system_reboot()
{
    watchdog_reboot(0, 0, 0);
}



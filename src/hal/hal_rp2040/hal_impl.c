#include "hal_impl.h"


#define mavlink_write_serial(buf, size)
    //tud_cdc_write(buf, size);           \
    //tud_cdc_write_flush()


// MAVLINK init
//tud_cdc_read_flush();
    //tud_cdc_write_flush();

static void core1_entry();

int hal_init()
{
    stdio_usb_init();

    // Initialize misc system stuff that isn't covered by any specific driver

    // Pin to sense if USB is connected or not
    gpio_init(PIN_VUSB_SENSE);
    gpio_set_dir(PIN_VUSB_SENSE, GPIO_IN);

    // Start second core
    multicore_launch_core1(core1_entry);

    // This is needed to allow second core to pause execution in primary core, which is necessary when writing to flash etc.
    multicore_lockout_victim_init();
}

bool usb_connected()
{
    return gpio_get(PIN_VUSB_SENSE) != 0;
}

void system_reboot()
{
    watchdog_reboot(0, 0, 0);
}


static void core1_entry() {
    while (1) {
        mavlink_driver_update();
    }
}
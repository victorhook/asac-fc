#include "asac_fc.h"

#ifdef LINUX
    #include "time.h"
    #include<sys/time.h>

    static uint32_t system_t0;

    void system_init()
    {
        struct timeval tv;
        gettimeofday(&tv,NULL);
        system_t0 = (uint32_t) (((long long)tv.tv_sec)*1000)+(tv.tv_usec/1000);
    }

    void system_reboot()
    {
        printf("Rebooting system... (Nothing really happens)\n");
    }

    uint32_t us_since_boot()
    {
        return ms_since_boot() * 1000;
    }

    uint32_t ms_since_boot()
    {
        struct timeval tv;
        gettimeofday(&tv,NULL);
        return (uint32_t) (((long long)tv.tv_sec)*1000)+(tv.tv_usec/1000) - system_t0;
    }
    bool usb_connected()
    {
        return false;
    }
#else
    #include "hardware/watchdog.h"
    #include "machine.h"

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

    uint32_t us_since_boot()
    {
        return (uint32_t) to_us_since_boot(get_absolute_time());
    }

    uint32_t ms_since_boot()
    {
        return to_ms_since_boot(get_absolute_time());
    }
#endif

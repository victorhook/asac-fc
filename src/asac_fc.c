#include "asac_fc.h"
#include "hardware/watchdog.h"


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

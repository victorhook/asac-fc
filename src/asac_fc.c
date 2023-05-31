#include "asac_fc.h"


uint32_t us_since_boot()
{
    return (uint32_t) to_us_since_boot(get_absolute_time());
}

uint32_t ms_since_boot()
{
    return to_ms_since_boot(get_absolute_time());
}

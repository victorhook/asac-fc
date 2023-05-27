#include "asac_fc.h"


uint64_t us_since_boot()
{
    return to_us_since_boot(make_timeout_time_us(time_us_64()));
}
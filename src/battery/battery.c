#include "battery.h"
#include "battery_adc.h"


battery_t bat1;


int battery_init()
{
    // Currently we only support ADC reading, but can have other backends here in future
    return battery_adc_init();
}

void battery_update()
{
    bat1.voltage_mv = battery_adc_read();
}
 
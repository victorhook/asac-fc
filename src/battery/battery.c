#include "battery.h"
#include "battery_adc.h"

extern float brd_bat_volt;
extern float brd_bat_curr;

battery_t bat1;


int battery_init()
{
    // Currently we only support ADC reading, but can have other backends here in future
    return battery_adc_init();
}

void battery_update()
{

}

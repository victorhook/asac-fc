#include "battery_adc.h"
#include "hal.h"


extern float brd_bat_volt;
extern float brd_bat_curr;
extern float brd_bat_volt_scaler;
extern float brd_bat_curr_scaler;

int battery_adc_init()
{
    int res = 0;

    if (brd_bat_volt != -1)
    {
        res |= hal_adc_init(brd_bat_volt);
    }
    if (brd_bat_curr != -1)
    {
        res |= hal_adc_init(brd_bat_curr);
    }

    return res;
}


float battery_adc_read()
{
    // TODO: Fix channel and scaler
    int raw_adc;
    hal_adc_read(0, &raw_adc);
    float scaler = 0.0011;
    return (float) raw_adc * scaler;
}


#include "hal_impl.h"



int hal_adc_init(const uint8_t pin)
{
    uint input;

    adc_gpio_init(pin);

    #if defined(HAL_RP2350B)
        input = pin - 40;
    #else
        input = pin - 26;
    #endif

    adc_select_input(input);

    return 0;
}

void hal_adc_read(const int channel, int* value)
{
    /// TODO: SUpport more channels?
    *value = adc_read();
}
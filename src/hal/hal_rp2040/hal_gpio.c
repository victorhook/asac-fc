#include "hal_impl.h"
#include <hardware/gpio.h>


int hal_gpio_init(const uint8_t pin, const hal_gpio_function_t function, const uint8_t default_value)
{
    gpio_init(pin);
    gpio_set_dir(pin, (function == HAL_GPIO_FUNCTION_OUTPUT) ? GPIO_OUT : GPIO_IN);
    gpio_put(pin, default_value);
}

void hal_gpio_set(const uint8_t pin, const uint8_t value)
{
    gpio_put(pin, value);
}
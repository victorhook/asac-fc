#include "led.h"
#include "hal.h"
#include "mavlink.h"
#include "mavlink_driver/mavlink_driver.h"

#define GREEN_OFF_RED_OFF() {gpio_put(PIN_LED_GREEN, 0); gpio_put(PIN_LED_RED, 0);}
#define GREEN_ON_RED_OFF()  {gpio_put(PIN_LED_GREEN, 1); gpio_put(PIN_LED_RED, 0);}
#define GREEN_OFF_RED_ON()  {gpio_put(PIN_LED_GREEN, 0); gpio_put(PIN_LED_RED, 1);}
#define GREEN_ON_RED_ON()   {gpio_put(PIN_LED_GREEN, 1); gpio_put(PIN_LED_RED, 1);}


int led_init()
{
    brd_led1 = 19;
    brd_led2 = 20;
    hal_gpio_init(brd_led1, HAL_GPIO_FUNCTION_OUTPUT, 0);
    hal_gpio_init(brd_led2, HAL_GPIO_FUNCTION_OUTPUT, 0);
    hal_gpio_init(brd_led3, HAL_GPIO_FUNCTION_OUTPUT, 0);


    return 0;
}


void led_on(const uint8_t pin)
{
    hal_gpio_set(pin, 1);
}

void led_off(const uint8_t pin)
{
    hal_gpio_set(pin, 0);
}

void led_blink(const uint8_t pin, const uint16_t on_time, const uint16_t off_time)
{

}

void led_run_boot_sequence() {
    /*
    for (int i = 0; i < 3; i++) {
        GREEN_ON_RED_OFF();
        hal_sleep_ms(150);
        GREEN_OFF_RED_ON();
        hal_sleep_ms(150);
    }
    GREEN_OFF_RED_OFF();
    */
}

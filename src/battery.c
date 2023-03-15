#include "battery.h"

#include "machine.h"

#define ADC_VREF 3.3
#define ADC_RANGE ((1 << 12) - 1)  // 12 bit ADC, 0 <-> 4095
#define ADC_CONVERT (ADC_VREF / ADC_RANGE)

static uint16_t vbat_adc_raw;


void battery_measure_init() {
    adc_init();
    adc_gpio_init(PIN_VBAT_ADC);
}


float battery_measure_read() {
    adc_select_input(VBAT_ADC_INPUT_NUMBER);
    vbat_adc_raw = adc_read();
    return vbat_adc_raw * ADC_CONVERT;
}


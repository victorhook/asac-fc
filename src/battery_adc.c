#include "battery_adc.h"
#include "machine.h"

#define ADC_VREF 3.3
#define ADC_RANGE ((1 << 12) - 1)  // 12 bit ADC, 0 <-> 4095

// R1 = 10k
// R2 = 5.6k
// Voltage divider factor = R2 / (R1 + R2)
//                        = 5.6 / (5.6 + 10) ≃ 0.36
#define VOLTAGE_DIVIDER_R1 10000
#define VOLTAGE_DIVIDER_R2 5600
#define VOLTAGE_DIVIDER_MULT_FACTOR ( (float)(1.0 / ( (float) VOLTAGE_DIVIDER_R2 / ((float) VOLTAGE_DIVIDER_R1 + (float) VOLTAGE_DIVIDER_R2) )) )
#define ADC_CONVERT (float) (ADC_VREF / ADC_RANGE)

// This conversion factor is found experimentally
#define VBAT_ADC_CONVERSION_MV (float) (0.002048502238502238 * 1000.0)

vbat_t vbat;


int battery_adc_init() {
    adc_init();
    adc_gpio_init(PIN_VBAT_ADC);
    return 0;
}


float battery_adc_read() {
    adc_select_input(VBAT_ADC_INPUT_NUMBER);
    vbat.raw = adc_read();
    vbat.scaledMv = (float) vbat.raw * VBAT_ADC_CONVERSION_MV;
    return vbat.scaledMv;
}


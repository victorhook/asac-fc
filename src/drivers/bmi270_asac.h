#ifndef BMI270_H
#define BMI270_H


#include <hardware/spi.h>


typedef struct {
    int result;
    spi_inst_t* spi;
    uint8_t cs_pin;
} bmi270_t;


int bmi270_asac_init(bmi270_t* bmi, spi_inst_t* spi, uint8_t cs_pin);


int bmi270_asac_read(bmi270_t* bmi, float acc[3], float gyro[3]);


#endif /* BMI270_H */

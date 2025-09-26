#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    char id[17];
    float* value;
} mav_param_t;


void read_parameters();

void write_parameters();

float get_param_value(const uint16_t index);

bool set_param_value(const char* param_id, const float param_value, const uint8_t param_type);

extern mav_param_t mav_params[];

extern const uint16_t nbr_of_parameters;

static inline const char* get_param_id(const uint16_t index) { return mav_params[index].id; }

#endif
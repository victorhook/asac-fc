
#ifndef SENSOR_H
#define SENSOR_H

#include <stdbool.h>

typedef struct
{
    bool present;
    bool enabled;
    bool healthy;
} sensor_t;

#endif
#ifndef UTIL_H
#define UTIL_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "lpf.h"

typedef struct
{
    union
    {
        struct { float x, y, z; };
        struct { float roll, pitch, yaw; };
        struct { float roll_rate, pitch_rate, yaw_rate; };
    };
} vec3f_t;

bool withinf(const float value, const float min_value, const float max_value);

bool within(const int value, const int min_value, const int max_value);

int constrain(int value, int from, int to);

float constrainf(float value, float from, float to);

int map(int value, int from_min, int from_max, int to_min, int to_max);

float mapf(float value, float from_min, float from_max, float to_min, float to_max);

int min(const int value1, const int value2);

float minf(const float value1, const float value2);

int max(const int value1, const int value2);

float maxf(const float value1, const float value2);

float clampf_low(const float value, const float low);

float clampf_high(const float value, const float high);

float clampf(const float value, const float low, const float high);

int clamp_low(const int value, const int low);

int clamp_high(const int value, const int high);

int clamp(const int value, const int low, const int high);

#endif

#include "util.h"


int constrain(int value, int from, int to)
{
    if (value <= from) return from;
    if (value >= to) return to;
    return value;
}

float constrainf(float value, float from, float to)
{
    if (value <= from) return from;
    if (value >= to) return to;
    return value;
}

int map(int value, int from_min, int from_max, int to_min, int to_max)
{
    if (from_max == from_min) return  to_min; // Prevent division by 0
    float alpha = (float) (value - from_min) / (float) (from_max - from_min);
    return to_min + (alpha * (to_max - to_min));
}

float mapf(float value, float from_min, float from_max, float to_min, float to_max)
{
    if (from_max == from_min) return  to_min; // Prevent division by 0
    float alpha = (value - from_min) / (from_max - from_min);
    return to_min + (alpha * (to_max - to_min));
}

int min(const int value1, const int value2)
{
    return (value1 < value2) ? value1 : value2;
}

float minf(const float value1, const float value2)
{
    return (value1 < value2) ? value1 : value2;
}

int max(const int value1, const int value2)
{
    return (value1 > value2) ? value1 : value2;
}

float maxf(const float value1, const float value2)
{
    return (value1 > value2) ? value1 : value2;
}

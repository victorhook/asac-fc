
#ifndef LPF_H
#define LPF_H

typedef struct
{
    float value;
    float alpha;
} lpf_t;

void lpf_init(lpf_t *lpf, const float alpha);

void lpf_init_with_value(lpf_t *lpf, const float alpha, const float init_value);

float lpf_update(lpf_t* lpf, const float value);

#endif
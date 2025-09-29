#include "lpf.h"


void lpf_init(lpf_t *lpf, const float alpha)
{
    lpf->value = 0;
    if (alpha < 0)
    {
        lpf->alpha = 0;
    }
    else if (alpha > 1)
    {
        lpf->alpha = 1;
    }
    else
    {
        lpf->alpha = alpha;
    }
}

void lpf_init_with_value(lpf_t *lpf, const float alpha, const float init_value)
{
    lpf_init(lpf, alpha);
    lpf->value = init_value;
}

float lpf_update(lpf_t *lpf, const float value)
{
    lpf->value = (lpf->alpha * value) + ((1 - lpf->alpha) * lpf->value);
    return lpf->value;
}

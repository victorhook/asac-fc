
#ifndef AHRS_H
#define AHRS_H

typedef enum
{
    AHRS_ORIENTATION_NONE = 0,
    AHRS_ORIENTATION_ROLL180 = 8,
} ahrs_orientation_t;

int ahrs_init();

void ahrs_update();


#endif
#ifndef SETTINGS_H
#define SETTINGS_H

#include "pid.h"


typedef struct {
    pid_state_t pid_roll;
    pid_state_t pid_pitch;
    pid_state_t pid_yaw;
    char craft_name[20];
    char version[3];
}__attribute__((packed)) settings_t;


int settings_read_from_flash(settings_t* settings);


int settings_write_to_flash(const settings_t* settings);


#endif /* SETTINGS_H */

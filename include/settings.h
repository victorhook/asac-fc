#ifndef SETTINGS_H
#define SETTINGS_H

#include "pid.h"
#include "mavlink_params.h"


typedef system_params_t system_settings_t;
extern system_settings_t system_settings;


int settings_init();


int settings_read_from_flash(system_settings_t* settings);


void settings_write_to_flash(const system_settings_t* settings);


void settings_reset_default();


#endif /* SETTINGS_H */

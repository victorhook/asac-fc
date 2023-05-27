#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "log.h"


int telemetry_init();


void telemetry_update();


void telemetry_send_state(const log_block_data_t* log_block, const log_type_t type);


#endif /* TELEMETRY_H */

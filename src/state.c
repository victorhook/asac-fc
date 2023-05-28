#include "state.h"


state_t state = {
    .is_armed = false,
    .is_connected = false,
    .can_run_motors = false,
    .mode = MODE_BOOTING,
};

#include "state.h"


state_t state = {
    .is_armed = false,
    .last_recieved_rx_packet = 0,
    .mode = MODE_BOOTING
};

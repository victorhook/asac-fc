#ifndef RECEIVER_H
#define RECEIVER_H

#include "asac_fc.h"
#include "rc/rc.h"


/*
 * Initializes the receiver.
 * Depending on which RX protocol is found in the settings, this
 * will initialize the correct RX protocol handler, eg IBUS/CRSF etc.
 * Returns 0 on success.
 */
int receiver_init();

/*
 * Fills `rx_state` with the curent state of the receiver
 * This state includes the latest received packet as well as statistics
 * of the RX link.
 */
void receiver_get_state(rx_state_t* rx_state);



#endif /* RECEIVER_H */

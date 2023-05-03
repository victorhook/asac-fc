#include "telemetry.h"
#include "vstp.h"

#include "string.h"
#include "pico/multicore.h"


static vstp_client_t vstp_client;
static control_update_flag;

static uint8_t tx[256];
static uint64_t log_block_unique_id;
static log_block_header_t header;

#define LOG_DATA_OFFSET sizeof(log_block_header_t)


static uint64_t us_since_boot()
{
    return to_us_since_boot(make_timeout_time_us(time_us_64()));
}


static void uart_write(const char* data, const uint16_t len) {

}


int telemetry_init() {
    log_block_unique_id = 0;
    control_update_flag = 0;
    vstp_init(&vstp_client, uart_write);
    return 0;
}

void telemetry_update() {
    if (!control_update_flag) {
        return;
    }

    //vstp_transmit(vstsp_client, VSTP_CMD_LOG_DATA, &tx, const uint16_t len)


    control_update_flag = 0;
}

void telemetry_send_state(const log_block_t* log_block, const log_type_t type) {
    header.id++;
    header.timestamp = us_since_boot();
    header.type = type;


    switch (type) {
        case LOG_TYPE_PID:
            memcpy(&tx[LOG_DATA_OFFSET], log_block, sizeof(log_block_control_loop_t));
            break;
    }

    control_update_flag = 1;
}

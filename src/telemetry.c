#include "telemetry.h"
#include "vstp.h"
#include "uart_tx.pio.h"
#include "asac_fc.h"
#include "machine.h"

#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <hardware/pio.h>


static vstp_client_t vstp_client;
static uint8_t control_update_flag;

static uint8_t tx[256];
static uint8_t tx_len;
static log_block_header_t header;

#define LOG_DATA_OFFSET sizeof(log_block_header_t)


/*
    Since rp2040 has low number of hardware UARTS,
    the telemetry module needs to use pio UART instead.
    In the future, the pio uart should probably be broken
    out to its separate module, but for now, it's all handled
    in this file.
*/

// -- PIO -- //
static PIO pio_inst = pio0;
static uint pio_sm_tx = 0;
const uint PIN_TX = PIN_TELEMETRY_TX;
const uint BAUDRATE = 921600;
//const uint BAUDRATE = 115200;

// Public methods from pio file:
// void uart_tx_program_init(PIO pio, uint sm, uint offset, uint pin_tx, uint baud)
// void uart_tx_program_write(PIO pio, uint sm, const uint8_t* data, const uint8_t len)

static int uart_pio_init();

static void uart_write(const uint8_t* data, const uint8_t len) {
    uart_tx_program_write(pio_inst, pio_sm_tx, data, len);
}

int telemetry_init() {
    header.id = 0;
    control_update_flag = 0;
    uart_pio_init();

    // Reset telemetry node and start logging
    vstp_init(&vstp_client, uart_write);
    vstp_transmit(&vstp_client, VSTP_CMD_RESET, 0, 0);
    vstp_transmit(&vstp_client, VSTP_CMD_LOG_START, 0, 0);

    return 0;
}

void telemetry_update() {
    if (!control_update_flag) {
        return;
    }

    vstp_transmit(&vstp_client, VSTP_CMD_LOG_DATA, (uint8_t*) &tx, tx_len);

    control_update_flag = 0;
}

void telemetry_send_state(const log_block_data_t* log_block, const log_type_t type) {
    header.id++;
    header.timestamp = us_since_boot();
    header.type = type;

    // Copy log block header
    memcpy(&tx, &header, sizeof(log_block_header_t));

    // Copy log block data
    switch (type) {
        case LOG_TYPE_PID:
            memcpy(&tx[LOG_DATA_OFFSET], log_block, sizeof(log_block_data_control_loop_t));
            break;
        default:
            // TODO: Handle
            break;
    }

    tx_len = sizeof(log_block_header_t) + sizeof(log_block_data_control_loop_t);

    control_update_flag = 1;
}


static int uart_pio_init() {
    uint offset = pio_add_program(pio0, &uart_tx_program);
    uart_tx_program_init(pio0, pio_sm_tx, offset, PIN_TX, BAUDRATE);

    return 0;
}

#include "receiver.h"
#include "machine.h"
#include "settings.h"
#include "rc/rc.h"
#include "rc/ibus.h"
#include "rc/crsf.h"

#include "stdbool.h"

#define UART    uart1
#define UART_HW uart1_hw

#define IBUS_BAUDRATE 115200
#define CRSF_BAUDRATE 420000

static rc_parse_byte     parse_byte;
static rc_get_last_state get_last_state;
static rc_scale_channel  scale_channel;


// RX interrupt handler
static void on_uart_rx();

static void init_uart(const uint32_t baudrate, const uart_parity_t parity);


int receiver_init() {
    rx_protocol_t rx_proto = (rx_protocol_t) system_params.rc_protocol.param_value;

    switch (rx_proto)
    {
        case RX_PROTOCOL_IBUS:
            parse_byte = ibus_parse_byte;
            get_last_state = ibus_get_last_state;
            scale_channel = ibus_scale_channel;
            init_uart(IBUS_BAUDRATE, UART_PARITY_EVEN);
            ibus_init();
            break;
        case RX_PROTOCOL_CRSF:
            parse_byte = crsf_parse_byte;
            get_last_state = crsf_get_last_state;
            scale_channel = crsf_scale_channel;
            init_uart(CRSF_BAUDRATE, UART_PARITY_NONE);
            crsf_init();
            break;
        default:
            printf("NO VALID RX PROTOCOL FOUND, %d!\n", system_params.rc_protocol);
            return -1;
            break;
    }

    return 0;
}

void receiver_get_state(rx_state_t* rx_state) {
    get_last_state(rx_state);
}


uint16_t receiver_scale_channel(const uint16_t raw) {
    return scale_channel(raw);
}


// -- Private -- //
static void on_uart_rx() {
    while (uart_is_readable(uart1)) {
        // Read 1 byte from UART buffer and give it to the RX protocol parser
        uint8_t byte = uart_getc(uart1);
        parse_byte(byte);
    }
}

static void init_uart(const uint32_t baudrate, const uart_parity_t parity) {
    uart_init(uart1, baudrate);
    gpio_set_function(PIN_RX1, GPIO_FUNC_UART);

    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, parity);
    uart_set_fifo_enabled(uart1, true);

    // Enable UART interrupt
    irq_set_exclusive_handler(UART1_IRQ, on_uart_rx);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(uart1, true, false);
}

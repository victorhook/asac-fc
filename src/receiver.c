#include "receiver.h"
#include "machine.h"
#include "ibus.h"

#include "pico/stdlib.h"


// RX interrupt handler
static void on_ibus_uart_rx() {
    while (uart_is_readable(uart1)) {
        uint8_t ch = uart_getc(uart1);
        ibus_process_byte(ch);
    }
}

static void init_ibus_uart()
{
    uart_init(uart1, 115200);
    gpio_set_function(PIN_RX1, GPIO_FUNC_UART);
    //gpio_set_function(5, GPIO_FUNC_UART);

    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_EVEN);
    uart_set_fifo_enabled(uart1, false);

    // Enable UART interrupt
    irq_set_exclusive_handler(UART1_IRQ, on_ibus_uart_rx);
    irq_set_enabled(UART1_IRQ, true);

    uart_set_fifo_enabled(uart1, true);

    uart_set_irq_enables(uart1, true, false);
}


int receiver_init() {
    init_ibus_uart();
    return 0;
}

void receiver_get_last_packet(rc_input_t* packet) {
    ibus_get_last_packet((ibus_packet_t*) packet);
}

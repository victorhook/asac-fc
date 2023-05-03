#include "uart_test.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "uart_tx.pio.h"
#include "uart_rx.pio.h"


static PIO pio = pio0;
static uint sm_tx = 0;
static uint sm_rx = 1;


void on_data() {
    io_ro_32 reg = pio->rxf[sm_rx];
    char c = uart_rx_read_byte(pio, sm_rx);
    pio_sm_clear_fifos(pio, sm_rx);

    uart_tx_program_putc(pio, sm_tx, c);
    pio_interrupt_clear(pio, 0);
}


int uart_test_init() {
    const uint PIN_TX = 5;
    const uint PIN_RX = 4;
    const uint BAUDRATE = 9600;

    uint offset = pio_add_program(pio0, &uart_tx_program);
    uart_tx_program_init(pio0, sm_tx, offset, PIN_TX, BAUDRATE);

    offset = pio_add_program(pio0, &uart_rx_program);
    uart_rx_program_init(pio0, sm_rx, offset, PIN_RX, BAUDRATE);

    // Enable interrupt handler
    irq_set_exclusive_handler(PIO0_IRQ_0, on_data);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);

    return 0;
}


void uart_test_update() {
    //uart_tx_program_puts(pio_tx, sm_tx, "You wrote: ");
    //uart_tx_program_putc(pio_tx, sm_tx, '\n');
}


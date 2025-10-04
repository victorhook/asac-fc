#include "hal_impl.h"

#include "hal.h"
#include "pico/stdio.h"
#include "tusb.h"
#include <hardware/timer.h>
#include <pico/stdio_usb.h>
#include <pico/time.h>

#define mavlink_write_serial(buf, size)
    //tud_cdc_write(buf, size);           \
    //tud_cdc_write_flush()


// MAVLINK init
//tud_cdc_read_flush();
    //tud_cdc_write_flush();

static uint32_t serial_available()
{
    // Manually update the TinyUSB task
    //tud_task();
    // Check if we're connected and data available
    //return tud_cdc_connected() && tud_cdc_available();
}


bool hal_write_param(const uint32_t param_size, const uint32_t crc, const uint8_t* buf)
{
    
}

bool hal_read_param(uint32_t param_size, uint32_t* crc, uint8_t* buf)
{

}

int hal_adc_init(const int channel)
{
    return 0;
}

void hal_adc_read(const int channel, int* value)
{

}

static void core1_entry();


int hal_do_init()
{
    return 0;

    // Initialize misc system stuff that isn't covered by any specific driver

    // Pin to sense if USB is connected or not
    gpio_init(PIN_VUSB_SENSE);
    gpio_set_dir(PIN_VUSB_SENSE, GPIO_IN);

    // Start second core
    multicore_launch_core1(core1_entry);

    // This is needed to allow second core to pause execution in primary core, which is necessary when writing to flash etc.
    multicore_lockout_victim_init();
}


bool usb_connected()
{
    //return gpio_get(PIN_VUSB_SENSE) != 0;
    return tud_cdc_connected();
}

void system_reboot()
{
    watchdog_reboot(0, 0, 0);
}


static void core1_entry()
{
}

#define UART    uart1
#define UART_HW uart1_hw

// RX interrupt handler
static void on_uart_rx();

//static void init_uart(const uint32_t baudrate, const uart_parity_t parity);


// -- Private -- //
static void on_uart_rx() {
    /*while (uart_is_readable(uart1)) {
        // Read 1 byte from UART buffer and give it to the RX protocol parser
        uint8_t byte = uart_getc(uart1);
        parse_byte(byte);
    }*/
}
/*
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
*/

void hal_sleep_us(const uint32_t us)
{
    sleep_us(us);
}

void hal_sleep_ms(const uint32_t ms)
{
    sleep_ms(ms);
}

uint32_t hal_millis()
{
    return time_us_32() / 1000;
}

uint32_t hal_micros()
{
    return time_us_32();
}

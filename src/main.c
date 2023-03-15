#include "msp.h"


msp_state_t msp_usb;


int kasdk = 9; // @mem
float kaka99 = 2.3; // @mem

void uart_tx(const uint8_t* data, const uint8_t len) {
    // Sent data through UART
}

void uart_rx() {
    uint8_t byte;
    msp_process_byte(&msp_usb, byte);
}

void setup() {
    msp_set_response_callback(&msp_usb, uart_tx);
}

void loop() {

}

int main() {
    kasdk++;
    return 0;
}
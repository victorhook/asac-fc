#include "asac_fc.h"
#include "msp.h"
#include "machine.h"
#include "motor.h"
#include "receiver.h"
#include "imu.h"
#include "vsrtos.h"
#include "controller.h"
#include "led.h"
#include "battery_adc.h"
#include "settings.h"
#include "state.h"
#include "pid_controller.h"

#include "pico/stdio.h"


static void init_driver(int (*init_function)(), const char* name);

static int init_result = 0;


int main() {
    stdio_init_all();

    // Initialize all drivers
    state.mode = MODE_BOOTING;
    printf("Booting up...\n");
    printf("Initializing drivers\n");

    // Initialize LED driver and blink led boot-up sequence
    init_driver(led_init, "Led");
    led_run_boot_sequence();

    // Turn RED led high to indicate we're booting
    led_set(LED_RED, 1);

    init_driver(settings_init, "Settings");
    init_driver(receiver_init, "Receiver");
    init_driver(battery_adc_init, "Battery ADC");
    init_driver(imu_init, "IMU");
    init_driver(pid_controller_init, "PID Ctrl");
    init_driver(controller_init, "Controller");
    init_driver(motors_init, "Motors");

    // Done booting
    led_set(LED_RED, 0);

    // -- Create tasks -- /
    vsrtos_create_task(controller_update, "Controller", 1000, 1);

    state.mode = MODE_IDLE;

    // -- Start scheduler -- /
    vsrtos_scheduler_start();

    // Should never reach this point
    while (1);

    return 0;
}


static void init_driver(int (*init_function)(), const char* name) {
    int res = init_function();
    printf("  Init: %s ", name);
    if (res == 0) {
        printf("OK\n");
    } else {
        printf("Error: %d\n", res);
    }

    init_result |= res;
}

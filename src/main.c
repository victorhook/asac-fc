#include "asac_fc.h"
#include "motor.h"
#include "receiver.h"
#include "imu.h"
#include "vsrtos.h"
#include "controller.h"
#include "led.h"
#include "battery_adc.h"
#include "settings.h"
#include "state.h"
#include "telemetry.h"
#include "serial_mavlink.h"

#include <pico/stdio.h>
#include <pico/multicore.h>


static void init_driver(int (*init_function)(), const char* name);

static void go_to_error_during_startup();

static void core1_entry();

static void print_system_params();

static int init_result = 0;


int main() {
    stdio_usb_init();
    system_init();

    // Initialize all drivers
    state.mode = MODE_BOOTING;
    printf("Booting up...\n");
    printf("Initializing drivers\n");

    // Initialize LED driver and blink led boot-up sequence
    init_driver(led_init, "Led");
    led_run_boot_sequence();

    // Turn RED led high to indicate we're booting
    led_set(LED_RED, 1);

    init_driver(settings_init,       "Settings");
    init_driver(receiver_init,       "Receiver");
    init_driver(battery_adc_init,    "Battery ADC");
    init_driver(imu_init,            "IMU");
    init_driver(controller_init,     "Controller");
    init_driver(motors_init,         "Motors");
    //init_driver(serial_mavlink_init, "Serial MAVlink");
    #ifdef TELEMETRY_LOGGING
        init_driver(telemetry_init,   "Telemetry");
    #endif

    // Done booting
    led_set(LED_RED, 0);
    led_set(LED_GREEN, 1);

    if (init_result != 0) {
        state.mode = MODE_ERROR;
        go_to_error_during_startup();
    }

    printf("Boot OK, using following settings:\n");
    print_system_params();

    // Create tasks
    //vsrtos_create_task(controller_debug,    "Controller debug", 10, 1);
    vsrtos_create_task(controller_update,     "Controller",       1000, 1);
    vsrtos_create_task(serial_mavlink_update, "Serial MAVlink",   10, 3);

    #ifdef TELEMETRY_LOGGING
        vsrtos_create_task(controller_telemetry, "Telemetry log", 100, 2);
        multicore_launch_core1(core1_entry);
    #endif

    //multicore_launch_core1(core1_entry);

    state.mode = MODE_IDLE;
    led_set(LED_BLUE, 1);

    // Start scheduler
    vsrtos_scheduler_start();

    // Should never reach this point
    while (1);

    return 0;
}


// -- Helper functions -- //

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

static void core1_entry() {
    while (1) {
        //telemetry_update();
    }
}

static void print_system_params()
{
    printf("  - pid_gyro_roll_p: %f\n", system_params.pid_gyro_roll_p.param_value);
    printf("  - pid_gyro_roll_i: %f\n", system_params.pid_gyro_roll_i.param_value);
    printf("  - pid_gyro_roll_d: %f\n", system_params.pid_gyro_roll_d.param_value);
    printf("  - pid_gyro_pitch_p: %f\n", system_params.pid_gyro_pitch_p.param_value);
    printf("  - pid_gyro_pitch_i: %f\n", system_params.pid_gyro_pitch_i.param_value);
    printf("  - pid_gyro_pitch_d: %f\n", system_params.pid_gyro_pitch_d.param_value);
    printf("  - pid_gyro_yaw_p: %f\n", system_params.pid_gyro_yaw_p.param_value);
    printf("  - pid_gyro_yaw_i: %f\n", system_params.pid_gyro_yaw_i.param_value);
    printf("  - pid_gyro_yaw_d: %f\n", system_params.pid_gyro_yaw_d.param_value);
    printf("  - rx_protocol: %f\n", system_params.rx_protocol.param_value);
    printf("\n");
}

static void go_to_error_during_startup() {
    while (1) {
        // TODO: Better error handling
        printf("Error occurred during startup: %d\n", init_result);
        led_set(LED_RED, 1);
        sleep_ms(500);
        led_set(LED_RED, 0);
        sleep_ms(500);
    }
}
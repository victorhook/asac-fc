#include "util.h"
#include "motor/motor.h"
#include "rc/receiver.h"
#include "imu/imu.h"
#include "control/controller.h"
#include "led/led.h"
#include "battery/battery_adc.h"
#include "param/param.h"
#include "state.h"
#include "mavlink_driver/mavlink_driver.h"
#include "serial/serial.h"

static void init_driver(int (*init_function)(), const char* name);

static int init_result = 0;

int main() {
    hal_init();

    // Initialize USB serial to allow messages to be sent/buffered to GCS
    serial_usb_init();

    // Initialize all drivers
    state.mode = MODE_BOOTING;
    printf("Booting up...\n");

    // Initialize LED driver and blink boot-up sequence
    init_driver(led_init, "Led");
    led_run_boot_sequence();
    led_set(LED_RED, 1);

    //read_parameters();

    init_driver(receiver_init,       "Receiver");
    init_driver(battery_adc_init,    "Battery ADC");
    init_driver(imu_init,            "IMU");
    init_driver(controller_init,     "Controller");
    init_driver(motors_init,         "Motors");
    init_driver(mavlink_driver_init, "Serial MAVlink");

    // Done booting
    led_set(LED_RED, 0);
    led_set(LED_GREEN, 1);
    state.mode = MODE_IDLE;

    while (1)
    {
        const uint32_t period_us = 1000;
        uint32_t next_loop = hal_micros();
        static uint64_t frame = 0;

        while (1)
        {
            if (frame % 1000 == 0)   // 1 Hz
            {

            }
            if (frame % 100 == 0)    // 10 Hz
            {

            }
            if (frame % 1 == 0)      // 100 Hz
            {

            }

            // 1 kHz
            //controller_pid_loop();

            // Wait for next loop
            int time_to_sleep = next_loop - hal_micros();
            printf("%lu\n", frame);
            
            if (time_to_sleep > 0)
            {
                hal_sleep_us(time_to_sleep);
            }
            else
            {
                printf("PLS\n");
            }

            next_loop += period_us;
            frame++;
        }
    }

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

#include "mavlink.h"
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

int main()
{
    hal_init();

    // Initialize USB serial to allow messages to be sent/buffered to GCS
    serial_usb_init();
    mavlink_driver_init();

    // Initialize all drivers
    state.mode = MODE_BOOTING;
    gcs_printf(MAV_SEVERITY_INFO, "ASAC Booting up");

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

    // Done booting
    led_set(LED_RED, 0);
    led_set(LED_GREEN, 1);
    state.mode = MODE_IDLE;

    gcs_printf(MAV_SEVERITY_INFO, "Drivers initialized, let's go!");

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
            if (frame % 10 == 0)      // 100 Hz
            {
            }
            mavlink_driver_update();

            // 1 kHz
            //controller_pid_loop();

            // Wait for next loop
            int time_to_sleep = next_loop - hal_micros();
            
            if (time_to_sleep > 0)
            {
                hal_sleep_us(time_to_sleep);
            }

            next_loop += period_us;
            frame++;
        }
    }

    return 0;
}


// -- Helper functions -- //
static void init_driver(int (*init_function)(), const char* name)
{
    int res = init_function();
    if (res == 0) {
        gcs_printf(MAV_SEVERITY_DEBUG, "Init %s OK", name);
    } else {
        gcs_printf(MAV_SEVERITY_ERROR, "Init %s: Error (%d)", name, res);
    }

    init_result |= res;
}

#include "msp.h"
#include "machine.h"
#include "motor.h"
#include "receiver.h"
#include "imu.h"
#include "vsrtos.h"
#include "controller.h"
#include "led.h"

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/stdio.h"


// Found empirically
const int INPUT_MIN = 995;
const int INPUT_MAX = 2011;

/*
    Maps the input value (~1000-2000) to a value between 5 and 10, which is
    needed for PWM.
    Returns float value between 5 and 10.
*/
float input_val_to_percentage(const uint16_t input_value) {
    float perc = (input_value - INPUT_MIN) / (float) (INPUT_MAX - INPUT_MIN);
    float duty_percentage = 5 + (perc * 5);
    return duty_percentage;
}

bool is_armed = false;
static int init_result = 0;

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


int main() {
    stdio_init_all();

    // Initialize LED driver and blink led boot-up sequence
    init_driver(led_init, "Led");
    led_run_boot_sequence();

    // Turn RED led high to indicate we're booting
    led_set(LED_RED, 1);

    // Initialize all drivers
    printf("Booting up...\n");
    printf("Initializing drivers\n");

    init_driver(receiver_init, "Receiver");
    //init_driver(motors_init, "Motors");
    init_driver(imu_init, "IMU");

    // Done booting
    led_set(LED_RED, 0);

    //vsrtos_create_task()

    rc_input_t rc_input;

    static int i = 0;
    uint64_t t0 = time_us_64();
    while (1) {
        /*
        receiver_get_last_packet(&rc_input);
        float p = rc_input.channels[2] / 200.0;
        if (p < 5) {
            p = 5;
        } else if (p > 10) {
            p = 10;
        }
        */
        //printf("Packet received: %d, %.2f\n", rc_input.channels[2], p);

        imu_reading_t imu_reading;

        uint64_t before = time_us_64();
        imu_read(&imu_reading);
        printf("Delay: %lld\n", time_us_64() - before);
        i++;

        sleep_ms(250);
        continue;

        uint64_t now = time_us_64();
        if ((now - t0) > 1000000) {
            printf("S: %d\n", i);
            t0 = now;
            i = 0;
        }

        //scan_bus();
        //scan_bus();
        led_set(LED_RED, 0);
        //sleep_ms(100);
        led_set(LED_RED, 1);
        //sleep_ms(300);
    }

    return 0;
}
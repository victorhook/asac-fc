#include "hal.h"
#include "motor.h"
#include "ahrs.h"
#include "rc.h"
#include "imu.h"
#include "controller.h"
#include "led.h"
#include "battery.h"
#include "state.h"
#include "param.h"
#include "mavlink_driver.h"
#include "controller.h"

#include "scheduler.h"

static void init_driver(int (*init_function)(), const char* name);

static int driver = 0;
static int init_result = 0;


void test_print()
{
    //gcs_printf(MAV_SEVERITY_INFO, "BUS: %d, FREQ: %d, SDA: %d, SCL: %d", hal_i2c1.nbr, hal_i2c1.config.freq, hal_i2c1.config.sda, hal_i2c1.config.scl);
}

task_t tasks[] =
{
    {.update = controller_pid_loop,   .name = "PID",     .loop_divider = 1},
    {.update = ahrs_update,           .name = "AHRS",    .loop_divider = 1},
    {.update = mavlink_driver_update, .name = "MAVLink", .loop_divider = 1},
    {.update = hal_serial_update,     .name = "Serial",  .loop_divider = 1},
    {.update = rc_update,             .name = "RC",      .loop_divider = 2},
    {.update = battery_update,        .name = "Battery", .loop_divider = 10},
    {.update = test_print,            .name = "TEST", .loop_divider = 1000}
};

int main()
{
    // Read params from EEPROM/FLASH - This must be done first, as much of the board-specific stuff is defined in params, instead of compile-time.
    params_init();

    // Initialize HAL - Serial, SPI, I2C etc. This also includes buffer initialization for the buses
    hal_init();
    
    // Initialize USB serial to allow messages to be sent/buffered to GCS
    mavlink_driver_init();

    // Initialize all drivers
    state.mode = MODE_BOOTING;
    gcs_printf(MAV_SEVERITY_INFO, "ASAC Booting up");

    // Initialize LED driver and blink boot-up sequence
    init_driver(led_init, "Led");
    led_run_boot_sequence();
    led1_on();

    // Initialize rest of drivers
    init_driver(battery_init,    "Battery");
    init_driver(imu_init,            "IMU");
    init_driver(controller_init,     "Controller");
    init_driver(motors_init,         "Motors");
    init_driver(rc_init,             "Receiver");
    init_driver(ahrs_init,             "AHRS");

    // At last we'll initialize the controller
    init_driver(controller_init,         "Controller");

    // Initialize scheduler
    scheduler_init(tasks, sizeof(tasks) / sizeof(task_t));

    // Done booting
    led1_off();
    led2_on();
    state.mode = MODE_IDLE;
    gcs_printf(MAV_SEVERITY_INFO, "Drivers initialized, let's go!");

    // Start scheduler, this will never return, let's go!
    scheduler_run();

    return 0;
}


// -- Helper functions -- //
static void init_driver(int (*init_function)(), const char* name)
{
    int res = init_function();

    if (res == 0) {
        gcs_printf(MAV_SEVERITY_DEBUG, "Init [OK] %s", name);
    } else {
        gcs_printf(MAV_SEVERITY_ERROR, "Init [ER] %s: (%d)", name, res);
    }

    driver++;

    // Update initresult bitmask
    init_result |= (res << driver);
}

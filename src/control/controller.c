#include "controller.h"
#include "hal.h"
#include "imu/imu.h"
#include "mavlink.h"
#include "mavlink_driver/mavlink_driver.h"
#include "motor/motor.h"
#include "state.h"
#include "led/led.h"
#include "log/log.h"
#include "math.h"
#include "rc/rc.h"
#include "battery/battery.h"
#include <string.h>
#include "control/attitude_rate_controller.h"
#include "control/throttle_controller.h"
#include "util.h"

// Params
extern float max_roll_rate;
extern float max_pitch_rate;
extern float max_yaw_rate;

extern float fltmode_channel;
extern float roll_channel;
extern float pitch_channel;
extern float yaw_channel;
extern float throttle_channel;
extern float arm_channel;
extern float arm_pwm;

extern float mot_pwm_min;
extern float mot_pwm_max;
extern float mot_spin_arm;
extern float mot_spin_max;

rc_target_t rc_desired;
motor_output_t motor_outputs;

vec3f_t target_rates = {0};
float target_throttle = 0;

uint32_t last_ctrl_update = 0;

// External, global variables
extern motor_output_t motor_command_test;

static int arm_check_result;
static uint32_t last_logged_arm_failed = 0;

#define ARMING_CHECK_RESULT_OK 0


static void rc_convert_to_desired(rc_target_t* target, const rc_input_t* rc_input);

static void estimate_attitude(const imu_reading_t* imu_filtered, const float ctrl_loop_dt_s, state_t* state);

static bool is_armed(const rc_input_t* rc_input_constrained);

static void on_rc_disconnect();
static void on_rc_connect();

static void on_usb_connect();
static void on_usb_disconnect();

static void on_arm();
static void on_disarm();

static void set_motor_output(motor_output_t* output, const uint16_t pwm);

static void print_rc_input();

static void run_arming_check() {/*TODO: FILL*/}

int controller_init()
{
    last_ctrl_update = hal_micros();

    int result = atrc_controller_init();

    return result;
}

static void handle_rc_connect_change();

static void handle_usb_connect_change();

static void handle_arming();


static void dump_motor_outputs()
{
    printf("M: %d, %d, %d, %d\n", motor_outputs.m1, motor_outputs.m2, motor_outputs.m3, motor_outputs.m4);
}
static void dump_pids()
{
    printf("Pid: P: %.3f, R: %.3f, Y: %.3f, T: %.3f\n", pid_roll.out, pid_pitch.out, pid_yaw.out, target_throttle);
}

void controller_pid_loop()
{
    // Average controller update time: ~300us, (measured experimentally)
    uint32_t ctrl_loop_started = hal_micros();
    float ctrl_loop_dt_s = (float) (ctrl_loop_started - last_ctrl_update) / 1000000.0;

    run_arming_check();

    // Map receiver data to desired rotation rates.
    rc_convert_to_desired(&rc_desired, &rc_input_scaled);

    handle_rc_connect_change();

    handle_usb_connect_change();

    handle_arming();

    if (state.armed || state.force_armed)
    {
        if (state.run_motor_test)
        {
            memcpy(&motor_outputs, &motor_command_test, sizeof(motor_output_t));
        }
        else
        {
            vec3f_t measured_rate = {.roll_rate = imu_filtered.gyro_x, .pitch_rate = imu_filtered.gyro_y, .yaw_rate = imu_filtered.gyro_z};
            
            // Run throttle controller
            throttle_control_update(&target_throttle, rc_desired.throttle);

            // Run attitude rate controller - Updates PIDs for roll/pitch/yaw using measured rates vs desired
            atrc_controller_update(&rc_desired.attitude_rate, &measured_rate, target_throttle, ctrl_loop_dt_s);
     
            // Run motor mixer - Pass PID values to motor mixer to get output for motors.
            motor_mixer_update(&motor_outputs, pid_roll.out, pid_pitch.out, pid_yaw.out, target_throttle);
        }
    }
    else
    {   // Disarmed, so we'll just set motor output to lowest value
        set_motor_output(&motor_outputs, mot_pwm_min);
    }

    dump_motor_outputs();
    dump_pids();

    // Set motor output
    set_all_motors_pwm(&motor_outputs);

    // Update timestamp with last update
    last_ctrl_update = hal_micros();
}



// -- Helper functions -- //
static void on_rc_disconnect()
{
    led3_off();
    state.rc_connected = false;
}

static void on_rc_connect()
{
    led3_on();
    state.rc_connected = true;
}

static void on_usb_connect()
{
    state.usb_connected = true;
}

static void on_usb_disconnect()
{
    state.usb_connected = false;
}

static void on_arm()
{
    led1_on();
    state.armed = true;
    atcr_controller_reset();
}

static void on_disarm()
{
    led1_off();
    state.armed = false;
    atcr_controller_reset();
}


static void rc_convert_to_desired(rc_target_t* target, const rc_input_t* rc_input)
{
    // Assuming values are between RC_CHANNEL_MIN and RC_CHANNEL_MAX
    target->attitude_rate.roll  = mapf(rc_get_channel(roll_channel),  mot_pwm_min, mot_pwm_max, -max_roll_rate,  max_roll_rate);
    target->attitude_rate.pitch = mapf(rc_get_channel(pitch_channel), mot_pwm_min, mot_pwm_max, -max_pitch_rate, max_pitch_rate);
    target->attitude_rate.yaw   = mapf(rc_get_channel(yaw_channel),   mot_pwm_min, mot_pwm_max, -max_yaw_rate,   max_yaw_rate);
    target->throttle            = mapf(rc_get_channel(throttle_channel), mot_pwm_min, mot_pwm_max, 0.0f, 1.0f);
}

static void set_motor_output(motor_output_t* output, const uint16_t pwm)
{
    output->m1 = pwm;
    output->m2 = pwm;
    output->m3 = pwm;
    output->m4 = pwm;
}


static void handle_rc_connect_change()
{
    // Check if we're connected (gotten radio packet within ~X ms)
    bool rc_connected = is_rc_connected(&rc_input_raw);
    if (rc_connected != state.rc_connected)
    {
        if (rc_connected)
        {
            on_rc_connect();
        }
        else
        {
            on_rc_disconnect();
        }
    }
}
static void handle_usb_connect_change()
{
    // Check if we're connected to USB
    bool is_usb_connected = usb_connected();
    if (is_usb_connected != state.usb_connected)
    {
        if (is_usb_connected) {
            on_usb_connect();
        } else {
            on_usb_disconnect();
        }
    }
}
static void handle_arming()
{
    if (rc_desired.armed != state.armed)
    {
        if (rc_desired.armed)
        {
            if (arm_check_result == ARMING_CHECK_RESULT_OK)
            {
                on_arm();
            }
            else if ((hal_millis() - last_logged_arm_failed) > 1000)
            {
                gcs_printf(MAV_SEVERITY_WARNING, "Arming check failed: %d\n", arm_check_result);
                last_logged_arm_failed = hal_millis();
            }
        }
        else
        {
            on_disarm();
        }
    }
}
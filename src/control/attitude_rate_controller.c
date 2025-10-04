#include "pid/pid.h"
#include "param/param.h"
#include "util.h"


extern float atc_rat_rll_p;
extern float atc_rat_rll_i;
extern float atc_rat_rll_d;
extern float atc_rat_rll_ff;
extern float atc_rat_rll_imax;
extern float atc_rat_pit_p;
extern float atc_rat_pit_i;
extern float atc_rat_pit_d;
extern float atc_rat_pit_ff;
extern float atc_rat_pit_imax;
extern float atc_rat_yaw_p;
extern float atc_rat_yaw_i;
extern float atc_rat_yaw_d;
extern float atc_rat_yaw_ff;
extern float atc_rat_yaw_imax;

pid_handle_t pid_roll;
pid_handle_t pid_pitch;
pid_handle_t pid_yaw;


int atrc_controller_init()
{
    memset(&pid_roll, 0, sizeof(pid_handle_t));
    pid_roll.Kp = atc_rat_rll_p;
    pid_roll.Ki = atc_rat_rll_i;
    pid_roll.Kd = atc_rat_rll_d;
    pid_roll.Kff = atc_rat_rll_ff;
    pid_roll.imax = atc_rat_rll_imax;

    memset(&pid_pitch, 0, sizeof(pid_handle_t));
    pid_pitch.Kp = atc_rat_pit_p;
    pid_pitch.Ki = atc_rat_pit_i;
    pid_pitch.Kd = atc_rat_pit_d;
    pid_pitch.Kff = atc_rat_pit_ff;
    pid_pitch.imax = atc_rat_pit_imax;

    memset(&pid_yaw, 0, sizeof(pid_handle_t));
    pid_yaw.Kp = atc_rat_yaw_p;
    pid_yaw.Ki = atc_rat_yaw_i;
    pid_yaw.Kd = atc_rat_yaw_d;
    pid_yaw.Kff = atc_rat_yaw_ff;
    pid_yaw.imax = atc_rat_yaw_imax;
    
    return 0;
}

void atrc_controller_update(const vec3f_t* desired, const vec3f_t* measured, const float throttle, const float dt_s)
{
    bool skip_integrator = throttle < 0.005;
    pid_update(&pid_roll,  measured->roll_rate,  desired->roll_rate,  skip_integrator, dt_s);
    pid_update(&pid_pitch, measured->pitch_rate, desired->pitch_rate, skip_integrator, dt_s);
    pid_update(&pid_yaw,   measured->yaw_rate,   desired->yaw_rate,   skip_integrator, dt_s);
}

void atcr_controller_reset()
{
    // TODO: Better way of this?
    atrc_controller_init();
}

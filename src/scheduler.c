#include "scheduler.h"
#include "hal.h"
#include "lpf.h"
#include "util.h"

extern float sched_loop_rate; // From parameters

static task_t*  _tasks = NULL;
static uint32_t _nbr_of_tasks = 0;
static uint64_t frame = 0;
static uint64_t next_frame_us = 0;

static uint32_t loop_rate;
static uint32_t loop_period_us;
static bool initialized = false;

static float cpu_load = 0;
static float cpu_load_min = 0;
static float cpu_load_max = 0;
static lpf_t cpu_load_filter;

static inline bool should_run_task(const task_t* task, const uint64_t current_frame)
{
    return (task->enabled) && (task->run_at_frame == current_frame);
}

static void init_task(task_t* task)
{
    task->run_at_frame = 1;
    task->last_start = 0;
    task->last_finish = 0;
    task->executions = 0;
    task->exec_time_min = UINT32_MAX;
    task->exec_time_max = 0;
    task->exec_time_sum = 0;
    task->exec_missed = 0;
    task->enabled = true;
}

float scheduler_cpu_load_min()
{
    return cpu_load_min;
}

float scheduler_cpu_load_max()
{
    return cpu_load_max;
}

float scheduler_cpu_load_avg()
{
    return cpu_load_filter.value;
}

void scheduler_init(task_t* tasks, const uint32_t nbr_of_tasks)
{
    if (initialized) return;

    cpu_load = 0;
    cpu_load_min = 100000;
    cpu_load_max = 0;
    lpf_init(&cpu_load_filter, 0.1);

    loop_rate = constrain(sched_loop_rate, SCHEDULER_LOOP_RATE_MIN, SCHEDULER_LOOP_RATE_MAX);
    loop_period_us = 1000000 / loop_rate;

    for (int i = 0; i < nbr_of_tasks; i++)
    {
        task_t* task = &tasks[i];
        init_task(task);
    }
    _tasks = tasks;
    _nbr_of_tasks = nbr_of_tasks;

    initialized = true;
}

void scheduler_run()
{
    frame = 0;
    next_frame_us = hal_micros();

    while (1)
    {
        next_frame_us += loop_period_us;
        frame++;

        for (int i = 0; i < _nbr_of_tasks; i++)
        {
            task_t* task = &_tasks[i];
            
            if (should_run_task(task, frame))
            {
                // Check if we're late?
                if (hal_micros() > next_frame_us)
                {
                    // We're late! Must skip task... Try again next frame
                    task->exec_missed++;
                    task->run_at_frame++;
                    continue;
                }

                task->last_start = hal_micros();
                task->update();
                task->last_finish = hal_micros();

                // Update time metrics
                uint32_t dt = task->last_finish - task->last_start;
                if (dt > task->exec_time_max)
                {
                    task->exec_time_max = dt;
                }
                if (dt < task->exec_time_min)
                {
                    task->exec_time_min = dt;
                }
                task->exec_time_sum += dt;
                task->executions++;

                // Schedule next time for the frame
                task->run_at_frame += task->loop_divider;
            }
        }

        // Calculate cpu load metrics and wait for next frame
        int time_to_sleep_us = next_frame_us - hal_micros();
        int busy_time_us = loop_period_us - ( (time_to_sleep_us > 0) ? time_to_sleep_us : 0 );

        cpu_load = (float) busy_time_us / (float) loop_period_us;
        if (cpu_load > cpu_load_max)
        {
            cpu_load_max = cpu_load;
        }
        if (cpu_load < cpu_load_min)
        {
            cpu_load_min = cpu_load;
        }
        lpf_update(&cpu_load_filter, cpu_load);

        if (time_to_sleep_us > 0)
        {
            if (time_to_sleep_us > loop_period_us)
            {
                time_to_sleep_us = loop_period_us;
            }
            hal_sleep_us(time_to_sleep_us);
        }
        
    }
}

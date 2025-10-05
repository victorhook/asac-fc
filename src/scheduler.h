
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>


#define SCHEDULER_LOOP_RATE_MAX     2000
#define SCHEDULER_LOOP_RATE_MIN     50



typedef void (*task_update_fn)();


typedef struct
{
    task_update_fn update;
    char           name[20];
    uint16_t       loop_divider;
    uint64_t       run_at_frame;
    uint64_t       last_start;
    uint64_t       last_finish;
    uint32_t       executions;
    uint32_t       exec_time_min;
    uint32_t       exec_time_max;
    uint64_t       exec_time_sum;
    uint32_t       exec_missed;
    bool           enabled;
} task_t;


void scheduler_init(task_t* tasks, const uint32_t nbr_of_tasks);

void scheduler_run();

static inline uint32_t task_avg_execution(task_t* task)
{
    return (task->executions == 0) ? 0 : task->exec_time_sum / task->executions;
}

unsigned long scheduler_get_frame();

float scheduler_cpu_load_min();

float scheduler_cpu_load_max();

float scheduler_cpu_load_avg();

#endif
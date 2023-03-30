#ifndef VSRTOS_H
#define VSRTOS_H


/*
    vsrtos - Very simple RTOS is a lightweight rtos used for embedded systems.
*/

#include "stdint.h"

typedef enum {
    VSRTOS_RESULT_OK = 0,
    VSRTOS_RESULT_NOT_ENOUGH_MEMORY
} vsrtos_result_t;

typedef void (*task_function)();

// uint32 Timestamps in us gives us:
//   0xffffffff / (10^6) ≃ 4294.97 seconds
//   4294.97 / 60 ≃ 71.58 minutes
typedef struct {
    task_function update;
    char          name[20];
    uint16_t      frequency;
    uint8_t       priority;
    uint32_t      delay_us;
    uint32_t      last_called;
    uint32_t      last_finished;
    uint16_t      times_executed;
    uint8_t       id;
} task_t;


/* Prints information about all active tasks. Mainly used for debugging. */
void printTasks();

/*
Creates a new task block on the heap.
@parameters:
    update: Function that will be called
    name: Name of task
    frequency: Call frequency of the task (in hertz)
    priority: Priority of the task, higher number means higher priority
*/
vsrtos_result_t vsrtos_create_task(task_function update, const char* name, const uint16_t frequency, const uint8_t priority);

/*
Starts the scheduler.
This call blocks and should never return.
*/
void vsrtos_scheduler_start();


#endif /* VSRTOS_H */

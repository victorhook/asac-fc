#include "vsrtos.h"

#include "string.h"
#include "stdbool.h"


// -- Platform specific definitions //
#ifdef ARDUINO
    #include <Arduino.h>
    #define currentTime()          millis()
    #define DEBUG_PRINT(msg)       Serial.print(msg)
    #define DEBUG_PRINTF(msg, ...) Serial.print(msg)
#endif
#ifdef RP2040
    #include "stdio.h"
    #include "stdlib.h"
    #include "pico/stdlib.h"
    #define yield()
    #define currentTime()              ( (uint64_t) (time_us_64() / 1000) )
    #define DEBUG_PRINTF(_Format, ...) printf(_Format, __VA_ARGS__)
    #define DEBUG_PRINT(msg)           printf(msg)
#endif
#ifdef LINUX
    #include "stdio.h"
    #include <sys/time.h>
    #include "malloc.h"

    #define USE_TIME_BIAS
    #define yield()
    #define DEBUG_PRINTF(_Format, ...) printf(_Format, __VA_ARGS__)
    #define DEBUG_PRINT(msg)           printf(msg)

    static uint64_t timeBias = 0;

    unsigned long currentTime() {
        struct timeval tv;
        gettimeofday(&tv,NULL);
        uint64_t now = (((uint64_t) tv.tv_sec)*1000)+(tv.tv_usec/1000);
        return now - timeBias;
    }

#endif

// -- Task block struct used for linked list //
struct _task_block_t {
    task_t task;
    struct _task_block_t* next;
};

typedef struct _task_block_t task_block_t;

// Head of linked list
static task_block_t* tasks_head = NULL;

// Number of tasks, also used to uniquely identify tasks
static uint8_t nbr_of_tasks;

static bool is_init = false;

#define IDLE_TASK NULL

// Helper functions
static task_t* getNextTask();
static task_block_t* getPrevTaskBlockWithHigherPriority(const uint8_t priority);
static vsrtos_result_t create_task(task_block_t* new_task_block, task_function update, const char* name, const uint16_t frequency, const uint8_t priority);


// -- Public functions -- //

void printTasks() {
    task_block_t* t = tasks_head;
    while (t != NULL) {
        DEBUG_PRINTF("[%d] %s - %d Hz, delay_ms: %d\n", t->task.priority, t->task.name, t->task.frequency, t->task.delay_ms);
        t = t->next;
    }
}

vsrtos_result_t vsrtos_create_task_static(task_block_t* task_block, task_function update, const char* name, const uint16_t frequency, const uint8_t priority) {
    return create_task(task_block, update, name, frequency, priority);
}

vsrtos_result_t vsrtos_create_task(task_function update, const char* name, const uint16_t frequency, const uint8_t priority) {
    // Create task struct and set update function, name, frequency etc
    task_block_t* new_task_block = (task_block_t*) malloc(sizeof(task_block_t));
    if (new_task_block == NULL) {
        return VSRTOS_RESULT_NOT_ENOUGH_MEMORY;
    }
    return create_task(new_task_block, update, name, frequency, priority);
}

void vsrtos_scheduler_start() {
    if (is_init) {
        DEBUG_PRINT("Scheduler already started!");
        return;
    }

    #ifdef USE_TIME_BIAS
        timeBias = currentTime();
    #endif

    is_init = true;
    DEBUG_PRINT("Scheduler starting\n");

    while (1) {
        task_t* next_task = getNextTask();

        if (next_task == IDLE_TASK) {
            // Not much to do, we're just idling.
            yield();
            continue;
        }

        next_task->last_called = currentTime();
        next_task->update();
        next_task->times_executed++;
        next_task->last_executed = currentTime();
        DEBUG_PRINTF(
            "[%d] last_called: %llu, last_exe: %llu Name: %s\n",
            next_task->priority,
            next_task->last_called,
            next_task->last_executed,
            next_task->name
        );
    }

}



// -- Private -- /
static vsrtos_result_t create_task(task_block_t* new_task_block, task_function update, const char* name, const uint16_t frequency, const uint8_t priority) {
    strcpy(new_task_block->task.name, name);
    new_task_block->task.update         = update;
    new_task_block->task.priority       = priority;
    new_task_block->task.frequency      = frequency;
    new_task_block->task.delay_ms       = 1000.0 / frequency;
    new_task_block->task.last_called    = 0;
    new_task_block->task.last_executed  = 0;
    new_task_block->task.times_executed = 0;
    new_task_block->task.id             = nbr_of_tasks;

    if (tasks_head == NULL) {
        tasks_head = new_task_block;
        tasks_head->next = NULL;
    } else {
        // Find the last task block in queue and attach newly created to tail of it.
        task_block_t* prev_block = getPrevTaskBlockWithHigherPriority(priority);

        if (prev_block == NULL) {
            // If previous block is null, it means we should insert the task FIRST in the list.
            new_task_block->next = tasks_head;
            tasks_head = new_task_block;
        } else {
            // Insert the task in between two tasks.
            new_task_block->next = prev_block->next;
            prev_block->next = new_task_block;
        }
    }

    nbr_of_tasks++;
    return VSRTOS_RESULT_OK;
}

static task_t* getNextTask() {
    if (tasks_head == NULL) {
        DEBUG_PRINT("HEAD IS NULL\n");
        return NULL;
    }

    uint64_t now = currentTime();
    task_block_t* curr = tasks_head;
    bool found_task_to_run = false;

    while (curr != NULL && !found_task_to_run) {
        uint64_t dt = now - curr->task.last_executed;

        if (dt > curr->task.delay_ms) {
            // Enough time has passed since this task executed, so it's time to execute this task.
            found_task_to_run = true;
        } else if (curr->task.times_executed == 0) {
            // Task has never been called!
            found_task_to_run = true;
        } else {
            curr = curr->next;
        }
    }

    if (found_task_to_run) {
        return &curr->task;
    } else {
        return IDLE_TASK;
    }

}

static task_block_t* getPrevTaskBlockWithHigherPriority(const uint8_t priority) {
    task_block_t* curr = tasks_head;
    task_block_t* prev = NULL;
    //printf("CURRENT: %s, prio: %d\n", curr->task.name, curr->task.priority);

    // As long as we haven't reached the end of the list OR, hit same/higher prio...
    while (curr != NULL && curr->task.priority > priority) {
        //printf("> CURRENT: %s, prio: %d\n", curr->task.name, curr->task.priority);
        prev = curr;
        curr = curr->next;
    }

    return prev;
}

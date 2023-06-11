#include "drivers/oneshot.h"
#include "oneshot.pio.h"
#include "machine.h"

#include <pico/stdlib.h>

/*
    Oneshot PIO implementation wrapper, supports oneshot125 and oneshot42.
    Each motor has its own PIO state machine
*/

// These values have been found experimentally by inspecting the pulse
// durations with a logic analyzer.
// You could definitely find these values mathematically as well, but meh... :)
#define ONESHOT_125_MIN_DELAY  7812
#define ONESHOT_125_MAX_DELAY  15624 // ONESHOT_125_MIN_DELAY * 2

#define ONESHOT_42_MIN_DELAY   2620
#define ONESHOT_42_MAX_DELAY   5243  // (ONESHOT_42_MIN_DELAY * 2) + 3

#define ONESHOT_PIO            pio1
#define ONESHOT_IRQ_NUM        PIO1_IRQ_0
#define ONESHOT_PIO_IRQ_SOURCE 0

static uint16_t min_delay;
static uint16_t max_delay;
static uint16_t pulse_width_delay;


int oneshot_init(const oneshot_type_t oneshot_type)
{
    switch (oneshot_type)
    {
        case ONESHOT_TYPE_125:
            min_delay = ONESHOT_125_MIN_DELAY;
            max_delay = ONESHOT_125_MAX_DELAY;
            break;
        case ONESHOT_TYPE_42:
            min_delay = ONESHOT_42_MIN_DELAY;
            max_delay = ONESHOT_42_MAX_DELAY;
            break;
    }

    pulse_width_delay = max_delay - min_delay;

    // Add PIO program
    uint program_offset = pio_add_program(ONESHOT_PIO, &oneshot_program);
    oneshot_program_init(ONESHOT_PIO, 0, program_offset, PIN_M1);
    oneshot_program_init(ONESHOT_PIO, 1, program_offset, PIN_M2);
    oneshot_program_init(ONESHOT_PIO, 2, program_offset, PIN_M3);
    oneshot_program_init(ONESHOT_PIO, 3, program_offset, PIN_M4);

    // Enable interrupts
    irq_set_enabled(ONESHOT_IRQ_NUM, true);
    pio_set_irq0_source_enabled(ONESHOT_PIO, ONESHOT_PIO_IRQ_SOURCE, true);

    return 0;
}


void oneshot_set(const uint8_t motor, const float throttle)
{
    // Calculate delay time, aka how long the pulse should be high.
    uint16_t delay = min_delay + (throttle * pulse_width_delay);
    ONESHOT_PIO->txf[motor] = delay;
}

void oneshot_apply()
{
    // Clear PIO interrupt to trigger the pulses.
    pio_interrupt_clear(ONESHOT_PIO, ONESHOT_PIO_IRQ_SOURCE);
}

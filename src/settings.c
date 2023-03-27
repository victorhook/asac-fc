#include "settings.h"
#include "hardware/flash.h"
#include "pico/stdlib.h"


int settings_read_from_flash(settings_t* settings) {
    return 0;
}

int settings_write_to_flash(const settings_t* settings) {
    /*
    int int_status = save_and_disable_interrupts();
    //flash_range_erase()
    //flash_range_program()
    restore_interrupts(int_status);
    */
    return 0;
}

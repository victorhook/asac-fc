#include "settings.h"
#include "machine.h"

#include <hardware/sync.h>
#include "hardware/flash.h"
#include "pico/stdlib.h"
#include "string.h"
#include "stdio.h"



#define FLASH_TARGET_OFFSET    (ASAC_FC_FLASH_SIZE - FLASH_SECTOR_SIZE)
#define FLASH_PAGES_PER_SECTOR (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE)

static int* page_nbr_ptr = (int*) (XIP_BASE + FLASH_TARGET_OFFSET);
static int settings_hash;
static int page_nbr;
static uint8_t flash_buf[FLASH_PAGE_SIZE];

// Default setting values
settings_t settings = {
    .pid_roll = {
        .Kp = 1,
        .Ki = 1,
        .Kd = 0
    },
    .pid_pitch = {
        .Kp = 1,
        .Ki = 1,
        .Kd = 0
    },
    .pid_yaw = {
        .Kp = 1,
        .Ki = 1,
        .Kd = 0
    },
    .craft_name = {'A', 'S', 'A', 'C', ' ', 'F', 'C'},
    .version = {'0', '0', '1'}
};


static int calculate_simple_hash(const settings_t* settings);


int settings_init() {
    page_nbr = *page_nbr_ptr;
    if (page_nbr > FLASH_PAGES_PER_SECTOR) {
        printf(
            "Flash page number invalid read: %d, must be between 0-%d\n",
            page_nbr,
            FLASH_PAGES_PER_SECTOR
        );
        printf("Setting page number to 0.\n");
        page_nbr = 0;
    }

    return 0;
}


int settings_read_from_flash(settings_t* settings) {
    return 0;
}

int settings_write_to_flash(const settings_t* settings) {
    // Calculate page number to write to next.
    page_nbr = (page_nbr + 1) % FLASH_PAGES_PER_SECTOR;
    settings_hash = calculate_simple_hash(settings);

    // Prepare settings buffer
    int offset = 0;
    memcpy(&flash_buf[offset], &page_nbr, sizeof(int));
    offset += sizeof(int);
    memcpy(&flash_buf[offset], &settings_hash, sizeof(int));
    offset += sizeof(int);
    memcpy(&flash_buf[offset], &settings, sizeof(settings_t));

    uint32_t flash_offset = FLASH_TARGET_OFFSET + (page_nbr * FLASH_TARGET_OFFSET);

    int int_status = save_and_disable_interrupts();
    flash_range_program(flash_offset, flash_buf, FLASH_PAGE_SIZE);
    restore_interrupts(int_status);
    return 0;
}


static int calculate_simple_hash(const settings_t* settings) {
    int hash = 0;
    for (int byte = 0; byte < sizeof(settings_t); byte++) {
        hash ^= ((uint8_t*) settings)[byte];
    }
    return hash;
}

#include "settings.h"
#include "machine.h"
#include "asac_fc.h"

#include <hardware/sync.h>
#include <hardware/flash.h>

// Inspired by: https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/


#define FLASH_TARGET_OFFSET    (ASAC_FC_FLASH_SIZE - FLASH_SECTOR_SIZE)
#define FLASH_PAGES_PER_SECTOR (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE)

static int* settings_hash_ptr     = (int*)        (XIP_BASE + FLASH_TARGET_OFFSET);
static settings_t* flash_settings = (settings_t*) (XIP_BASE + FLASH_TARGET_OFFSET + 4);

static int settings_hash;
static uint8_t flash_buf[FLASH_PAGE_SIZE];

settings_t* settings;


static int calculate_simple_hash(const settings_t* settings);


int settings_init() {
    settings = &system_params;
    settings_hash = *settings_hash_ptr;

    int hash = calculate_simple_hash(flash_settings);

    if (hash != settings_hash) {
        printf("Hash calculation for settings failed, using default...!\n");
        printf("Expected %d, got: %d\n", hash, settings_hash);
        settings_write_to_flash(settings);
    } else {
        memcpy(settings, flash_settings, sizeof(settings_t));
    }

    return 0;
}

int settings_write_to_flash(const settings_t* ASD) {
    settings_hash = calculate_simple_hash(ASD);

    // Prepare settings buffer
    memcpy(&flash_buf[0], &settings_hash, 4);
    memcpy(&flash_buf[4], ASD, sizeof(settings_t));

    int int_status = save_and_disable_interrupts();
    // For some reason we need to erase the flash before writing, not sure why.
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, flash_buf, FLASH_PAGE_SIZE);
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

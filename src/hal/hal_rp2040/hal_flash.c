#include "hal.h"
#include <hardware/sync.h>
#include <hardware/flash.h>
#include "util.h"

// 16 MB
#define BOARD_FLASH_SIZE        (16 * 1024 * 1024)

// Inspired by: https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/

#define FLASH_TARGET_OFFSET    (BOARD_FLASH_SIZE - FLASH_SECTOR_SIZE)
#define FLASH_PAGES_PER_SECTOR (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE)

#define SETTINGS_NBR_OF_FLASH_PAGES ((sizeof(system_settings_t) / FLASH_PAGE_SIZE) + 1)
#define SETTINGS_FLASH_SIZE (SETTINGS_NBR_OF_FLASH_PAGES * FLASH_PAGE_SIZE)

static int* params_size_ptr     = (int*)     (XIP_BASE + FLASH_TARGET_OFFSET + 0);
static int* params_crc_ptr      = (int*)     (XIP_BASE + FLASH_TARGET_OFFSET + 4);
static uint8_t* params_data_ptr = (uint8_t*) (XIP_BASE + FLASH_TARGET_OFFSET + 8);


bool hal_write_param(const uint32_t param_size, const uint32_t crc, const uint8_t* buf)
{
    
}

bool hal_read_param(uint32_t* param_size, uint32_t* crc, uint8_t* buf)
{
    *param_size = *params_size_ptr;
    *crc = *params_crc_ptr;
    memcpy(buf, params_data_ptr, min(*param_size, MAX_PARAMS_STORAGE_DATA_SIZE));
    return true;
}

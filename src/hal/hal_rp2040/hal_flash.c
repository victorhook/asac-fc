#include "hal.h"
#include <hardware/sync.h>
#include <hardware/flash.h>
#include "util.h"

#define BOARD_FLASH_SIZE        (2 * 1024 * 1024)

// Inspired by: https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/

#define FLASH_TARGET_OFFSET    (BOARD_FLASH_SIZE - FLASH_SECTOR_SIZE)
#define FLASH_PAGES_PER_SECTOR (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE)


static uint32_t* params_size_ptr = (uint32_t*) (XIP_BASE + FLASH_TARGET_OFFSET + 0);
static uint32_t* params_crc_ptr  = (uint32_t*) (XIP_BASE + FLASH_TARGET_OFFSET + 4);
static uint8_t* params_data_ptr  = (uint8_t*)  (XIP_BASE + FLASH_TARGET_OFFSET + 8);


bool hal_write_param(const uint32_t param_size, const uint32_t crc, const uint8_t* buf)
{
    if (param_size > MAX_PARAMS_STORAGE_DATA_SIZE) return false;

    uint32_t ints = save_and_disable_interrupts();

    // Erase 1 sector (required before we write), on pico this is 4096 bytes
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);

    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, FLASH_PAGE_SIZE);

    // Copy size and crc into first part of page.
    memcpy(&page[0], &param_size, 4);
    memcpy(&page[4], &crc, 4);

    int offset = FLASH_PAGE_SIZE - 8;
    int first_page_data = min(offset, (int) param_size);
    memcpy(&page[8], buf, first_page_data);
    // Write first page, which includes the header
    flash_range_program(FLASH_TARGET_OFFSET, page, FLASH_PAGE_SIZE);

    int pages_written = 1;
    while (offset < param_size)
    {
        int bytes_left = param_size - offset;
        // Copy into page buffer
        memcpy(page, &buf[offset], min(bytes_left, FLASH_PAGE_SIZE));
        flash_range_program(FLASH_TARGET_OFFSET+(pages_written*FLASH_PAGE_SIZE), page, FLASH_PAGE_SIZE);
        offset += FLASH_PAGE_SIZE;
        pages_written++;
    }

    restore_interrupts(ints);
}

bool hal_read_param(uint32_t* param_size, uint32_t* crc, uint8_t* buf)
{
    *param_size = *params_size_ptr;
    *crc = *params_crc_ptr;
    int size = min(*param_size, MAX_PARAMS_STORAGE_DATA_SIZE);
    if (size < 0) return false; // If flash is erased it's usually filled with 0xFF, which is unsigned -1...
    memcpy(buf, params_data_ptr, size);
    return true;
}

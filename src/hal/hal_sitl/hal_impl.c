#include "hal_impl.h"

#include <time.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <libgen.h>   // for dirname()
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <dirent.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdlib.h>

#include "util.h"


static inline uint64_t micros64();

static struct timespec start_time;


int hal_adc_init(const int channel)
{
    return 0;
}

void hal_adc_read(const int channel, int* value)
{

}



int hal_do_init()
{
    // Capture program start as "time zero"
    clock_gettime(CLOCK_MONOTONIC, &start_time);

    return 0;
}

int hal_gpio_init(const uint8_t pin, const hal_gpio_function_t function, const uint8_t value)
{
    return 0;
}

uint32_t hal_micros(void) {
    return (uint32_t)micros64();  // wraps at ~71 min
}

uint32_t hal_millis(void) {
    return (uint32_t)(micros64() / 1000ULL);  // wraps at ~49.7 days
}

void hal_sleep_us(uint32_t us) {
    struct timespec req, rem;
    req.tv_sec  = us / 1000000U;
    req.tv_nsec = (us % 1000000U) * 1000U;

    while (nanosleep(&req, &rem) == -1 && errno == EINTR) {
        req = rem;
    }
}

void hal_sleep_ms(uint32_t ms) {
    hal_sleep_us(ms * 1000U);
}


static inline uint64_t micros64()
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    int64_t sec  = now.tv_sec  - start_time.tv_sec;
    int64_t nsec = now.tv_nsec - start_time.tv_nsec;

    if (nsec < 0) {
        sec  -= 1;
        nsec += 1000000000L;
    }

    return (uint64_t)sec * 1000000ULL + (uint64_t)(nsec / 1000ULL);
}

bool hal_write_param(const uint32_t param_size, const uint32_t crc, const uint8_t* buf)
{
    FILE* f = fopen("eeprom.bin", "wb");
    if (!f)
    {
        printf("Failed to open eeprom.bin!");
        return false;
    }

    fwrite((uint8_t*) &param_size, sizeof(uint32_t), 1, f);
    fwrite((uint8_t*) &crc, sizeof(uint32_t), 1, f);
    fwrite(buf, param_size, 1, f);

    fclose(f);

    return true;
}

bool hal_read_param(uint32_t param_size, uint32_t* crc, uint8_t* buf)
{
    FILE* f = fopen("eeprom.bin", "rb");
    if (!f)
    {
        printf("Failed to open eeprom.bin!");
        return false;
    }

    uint32_t size;
    fread(&size, sizeof(uint32_t), 1, f);
    if (size != param_size) return false;

    fread(crc, sizeof(uint32_t), 1, f);
    fread(buf, max(param_size, MAX_EEPROM_SIZE), 1, f);

    fclose(f);

    return true;
}

// -- Dummys -- //
bool usb_connected()
{
    return true;
}

void system_reboot()
{
    
}


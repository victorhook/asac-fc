cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

## -- Pico
set(PICO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/vendor/pico-sdk")
set(PICO_BOARD pico CACHE STRING "Board type")

message("PICO SDK IS: ${PICO_SDK_PATH}")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(asac-fc
  ${PROJECT_SOURCE_DIR}/src/hal/hal_rp2040/uart_tx.pio
  ${PROJECT_SOURCE_DIR}/src/hal/hal_rp2040/uart_rx.pio
  ${PROJECT_SOURCE_DIR}/src/hal/hal_rp2040/oneshot.pio
  ${PROJECT_SOURCE_DIR}/src/hal/hal_rp2040/hal_impl.c
)

# Add compiler flags to the target
# -Wno-address-of-packed-member: Known from MAVLINK. Hopefully doesn't crash anything.
target_compile_options(asac-fc PRIVATE -Wno-address-of-packed-member)

# Add PIO programs
pico_generate_pio_header(asac-fc ${CMAKE_CURRENT_LIST_DIR}/src/hal/hal_rp2040/uart_tx.pio)
pico_generate_pio_header(asac-fc ${CMAKE_CURRENT_LIST_DIR}/src/hal/hal_rp2040/uart_rx.pio)
pico_generate_pio_header(asac-fc ${CMAKE_CURRENT_LIST_DIR}/src/hal/hal_rp2040/oneshot.pio)

add_definitions(
  # 16MB: 16 * 1024 * 1024 = 16777216
  -DPICO_FLASH_SIZE_BYTES=16777216
)

pico_set_program_name(asac-fc "asac-fc")
pico_set_program_version(asac-fc "0.1")

# Enable USB serial port
pico_enable_stdio_uart(asac-fc 0)
pico_enable_stdio_usb(asac-fc 1)


# Add any user requested libraries
target_link_libraries(asac-fc
  asac-core
  pico_stdlib
  pico_multicore
  hardware_i2c
  hardware_spi
  hardware_adc
  hardware_flash
  hardware_sync
  hardware_pwm
  hardware_pio
)

pico_add_extra_outputs(asac-fc)


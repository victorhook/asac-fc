# HAL-specific executable
add_executable(asac-fc
  ${PROJECT_SOURCE_DIR}/src/hal/hal_rp2040/uart_tx.pio
  ${PROJECT_SOURCE_DIR}/src/hal/hal_rp2040/uart_rx.pio
  ${PROJECT_SOURCE_DIR}/src/hal/hal_rp2040/oneshot.pio
  ${PROJECT_SOURCE_DIR}/src/hal/hal_rp2040/hal_impl.c
)

# Add PIO programs
pico_generate_pio_header(asac-fc ${CMAKE_CURRENT_LIST_DIR}/src/hal/hal_rp2040/uart_tx.pio)
pico_generate_pio_header(asac-fc ${CMAKE_CURRENT_LIST_DIR}/src/hal/hal_rp2040/uart_rx.pio)
pico_generate_pio_header(asac-fc ${CMAKE_CURRENT_LIST_DIR}/src/hal/hal_rp2040/oneshot.pio)

target_compile_definitions(core PUBLIC HAL_RP2040)
target_compile_definitions(asac-fc PUBLIC PICO_FLASH_SIZE_BYTES=16777216)   # 16MB: 16 * 1024 * 1024 = 16777216

add_compile_definitions(
  RP2040 # Used by VSRTOS
  # Uncomment desired defines to remove
  #USE_DEFAULT_SETTINGS
  #LINUX
)

pico_set_program_name(asac-fc "asac-fc")
pico_set_program_version(asac-fc "0.1")

# Enable USB serial port
pico_enable_stdio_uart(asac-fc 0)
pico_enable_stdio_usb(asac-fc 1)



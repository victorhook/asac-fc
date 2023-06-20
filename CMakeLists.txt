# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/victor/coding/tools/pico/pico-sdk")

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(asac-fc C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(asac-fc
  src/main.c
  src/battery_adc.c
  src/settings.c
  src/pid.c
  src/ibus.c
  src/imu.c
  src/receiver.c
  src/controller.c
  src/motor.c
  src/led.c
  src/controller.c
  src/vsrtos.c
  src/state.c
  src/vstp.c
  src/asac_fc.c
  src/telemetry.c
  # Drivers
  src/drivers/mpu6050.c
  src/drivers/bmi270_asac.c
  src/drivers/oneshot.c
  # Vendor - BMI270
  vendor/BMI270-Sensor-API/bmi2_ois.c
  vendor/BMI270-Sensor-API/bmi270.c
  vendor/BMI270-Sensor-API/bmi2.c
  # PIO programs
  include/drivers/uart_tx.pio
  include/drivers/uart_rx.pio
  include/drivers/oneshot.pio
)

pico_generate_pio_header(asac-fc ${CMAKE_CURRENT_LIST_DIR}/include/drivers/uart_tx.pio)
pico_generate_pio_header(asac-fc ${CMAKE_CURRENT_LIST_DIR}/include/drivers/uart_rx.pio)
pico_generate_pio_header(asac-fc ${CMAKE_CURRENT_LIST_DIR}/include/drivers/oneshot.pio)


add_compile_definitions(
  RP2040
)

pico_set_program_name(asac-fc "asac-fc")
pico_set_program_version(asac-fc "0.1")

pico_enable_stdio_uart(asac-fc 0)
pico_enable_stdio_usb(asac-fc 1)

# Add the standard library to the build
target_link_libraries(asac-fc
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(asac-fc PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/vendor/BMI270-Sensor-API/
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(asac-fc
  hardware_i2c
  hardware_spi
  hardware_adc
  hardware_flash
  hardware_sync
  hardware_pwm
  hardware_pio
  pico_multicore
)

pico_add_extra_outputs(asac-fc)


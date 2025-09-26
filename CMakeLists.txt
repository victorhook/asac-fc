cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/vendor/pico-sdk")
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(${CMAKE_CURRENT_LIST_DIR}/src/hal/hal_rp2040/pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(asac-fc C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(asac-fc
  src/main.c
  src/battery/battery_adc.c
  src/pid/pid.c
  src/control/controller.c
  # Motor
  src/motor/oneshot.c
  src/motor/motor.c
  src/led/led.c
  src/vsrtos.c
  src/state.c
  src/util/util.c
  src/mavlink_driver/serial_mavlink.c
  src/parameter/mavlink_params.c
  src/parameter/settings.c
  # Utils
  src/util/ringbuf.c
  # RX
  src/rc/receiver.c
  src/rc/ibus.c
  src/rc/crsf.c
  # IMU
  src/imu/imu.c
  #src/imu/mpu6050.c
  #src/imu/bmi270_asac.c
  # Vendor - BMI270
  vendor/BMI270-Sensor-API/bmi2_ois.c
  vendor/BMI270-Sensor-API/bmi270.c
  vendor/BMI270-Sensor-API/bmi2.c
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
  -DCMAKE_BUILD_TYPE=DEBUG
  -DCMAKE_C_FLAGS_DEBUG="-g -O0 -Wall"
  -DCMAKE_CXX_FLAGS_DEBUG="-g -O0"
)

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

# Add the standard include files to the build
target_include_directories(asac-fc PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/src
  ${CMAKE_CURRENT_LIST_DIR}/src/hal
  ${CMAKE_CURRENT_LIST_DIR}/src/util
  ${CMAKE_CURRENT_LIST_DIR}/vendor/BMI270-Sensor-API/
  ${CMAKE_CURRENT_LIST_DIR}/vendor/c_library_v2/
  ${CMAKE_CURRENT_LIST_DIR}/vendor/c_library_v2/ardupilotmega
)

# Add any user requested libraries
target_link_libraries(asac-fc
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


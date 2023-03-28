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
  src/battery.c
  src/settings.c
  src/pid_controller.c
  src/pid.c
  src/ibus.c
  src/imu.c
  src/receiver.c
  src/controller.c
  src/motor.c
  src/motor_mixer.c
  src/led.c
  src/drivers/mpu6050.c
)

pico_set_program_name(asac-fc "asac-fc")
pico_set_program_version(asac-fc "0.1")

#pico_enable_stdio_uart(asac-fc 0)
#pico_enable_stdio_usb(asac-fc 1)

# Add the standard library to the build
target_link_libraries(asac-fc
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(asac-fc PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(asac-fc
        hardware_i2c
        hardware_adc
        hardware_flash
        hardware_sync
        hardware_pwm
)

pico_add_extra_outputs(asac-fc)


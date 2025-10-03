cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
set(HAL "RP2040" CACHE STRING "Which HAL to use")

message("CMAKE is using HAL: ${HAL}")

if ("${HAL}" STREQUAL "RP2040")
    ## Pull in pico cmake, must be done before project()
    set(PICO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/vendor/pico-sdk")
    set(PICO_BOARD pico CACHE STRING "Board type")

    # Pull in Raspberry Pi Pico SDK (must be before project)
    include(${CMAKE_CURRENT_LIST_DIR}/src/hal/hal_rp2040/pico_sdk_import.cmake)

    if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
        message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
    endif()
endif()


project(asac-fc C CXX ASM)
enable_testing()
include(CTest)

if ("${HAL}" STREQUAL "RP2040")
    # Initialise the Raspberry Pi Pico SDK, must be done after project()
    pico_sdk_init()
endif()

add_library(asac-core
    src/main.c
    src/pid/pid.c
    # Control
    src/control/attitude_rate_controller.c
    src/control/throttle_controller.c
    src/control/controller.c
    # Motor
    src/motor/oneshot.c
    src/motor/motor.c
    src/led/led.c
    src/state.c
    src/mavlink_driver/mavlink_driver.c
    # Utils
    src/util/util.c
    src/util/ringbuf.c
    src/util/lpf.c
    # AHRS
    src/ahrs/ahrs.c
    # Battery
    src/battery/battery.c
    src/battery/battery_adc.c
    # RX
    src/rc/rc.c
    src/rc/ibus.c
    src/rc/crsf.c
    # Params
    src/param/param.c
    # HAL
    src/hal/hal.c
    # IMU
    src/imu/imu.c
    src/imu/imu_mpu6050.c
    src/imu/imu_bmi270.c
    src/imu/imu_sitl.c
    # Scheduler
    src/scheduler.c
    # Vendor - BMI270
    vendor/BMI270-Sensor-API/bmi2_ois.c
    vendor/BMI270-Sensor-API/bmi270.c
    vendor/BMI270-Sensor-API/bmi2.c
)

# Add compiler flags to the target
# -Wno-address-of-packed-member: Known from MAVLINK. Hopefully doesn't crash anything.
target_compile_options(asac-core PRIVATE -Wno-address-of-packed-member)

# Add the standard include files to the build
target_include_directories(asac-core PUBLIC
  src
  src/hal
  src/util
  src/ahrs
  src/battery
  src/control
  src/drivers
  src/hal
  src/imu
  src/led
  src/log
  src/mavlink_driver
  src/motor
  src/param
  src/pid
  src/rc
  src/sensors
  src/serial4way
  src/util
  vendor/BMI270-Sensor-API/
  vendor/c_library_v2/
  vendor/c_library_v2/ardupilotmega
)

target_compile_definitions(asac-core PUBLIC HAL_${HAL}=1)

if ("${HAL}" STREQUAL "RP2040")  # -- RP2040 -- #
    add_executable(asac-fc
        src/hal/hal_rp2040/uart_tx.pio
        src/hal/hal_rp2040/uart_rx.pio
        src/hal/hal_rp2040/oneshot.pio
        src/hal/hal_rp2040/hal_impl.c
        src/hal/hal_rp2040/hal_i2c.c
        src/hal/hal_rp2040/hal_spi.c
        src/hal/hal_rp2040/hal_serial.c
        src/hal/hal_rp2040/hal_gpio.c
    )

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

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(asac-fc)

elseif("${HAL}" STREQUAL "SITL")  # -- SITL -- #

    add_executable(asac-fc
        src/hal/hal_sitl/hal_impl.c
        src/hal/hal_sitl/hal_serial.c
        src/hal/hal_sitl/hal_spi.c
        src/hal/hal_sitl/hal_i2c.c
        src/hal/hal_sitl/hal_gpio.c
    )

    target_link_libraries(asac-fc
        asac-core
        m
    )

    # Unity framework
    add_library(unity vendor/unity/src/unity.c)
    target_include_directories(unity PUBLIC vendor/unity/src)

    # Test: PID
    add_executable(test_pid tests/test_pid.c src/pid/pid.c)
    target_link_libraries(test_pid PRIVATE asac-core unity m)
    add_test(NAME PID COMMAND test_pid)

    # Test: Util
    add_executable(test_util tests/test_util.c src/util/util.c)
    target_link_libraries(test_util PRIVATE asac-core unity m)
    add_test(NAME UTIL COMMAND test_util)

    # Test: Ringbuf
    add_executable(test_ringbuf tests/test_ringbuf.c src/util/ringbuf.c)
    target_link_libraries(test_ringbuf PRIVATE asac-core unity m)
    add_test(NAME RINGBUF COMMAND test_ringbuf)

endif()


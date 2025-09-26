function(init_hal)

    # Pull in Raspberry Pi Pico SDK (must be before project)
    include(${CMAKE_CURRENT_SOURCE_DIR}/src/hal/hal_rp2040/pico_sdk_import.cmake)

    message("SRCDIR: ${CMAKE_CURRENT_SOURCE_DIR}")

    if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
        message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
    endif()

    # Initialise the Raspberry Pi Pico SDK
    pico_sdk_init()

endfunction()

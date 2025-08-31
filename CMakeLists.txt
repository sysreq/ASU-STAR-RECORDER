# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(air_data_system C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(air_data_system 
    src/main.cpp
    src/drivers/gps/gps_driver.cpp
    src/drivers/sensors/icm20948_driver.cpp
    src/drivers/sensors/bmp581_driver.cpp
    src/drivers/sensors/hx711_driver.cpp
    src/drivers/sensors/pitot_tube.cpp
    src/drivers/sdcard/sdcard.cpp
)

add_subdirectory(lib/sh2)
add_subdirectory(lib/sdcard)

target_include_directories(air_data_system PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/config
    ${CMAKE_CURRENT_LIST_DIR}/lib
)

pico_set_program_name(air_data_system "ADS")
pico_set_program_version(air_data_system "0.1")

pico_enable_stdio_uart(air_data_system 0)
pico_enable_stdio_usb(air_data_system 1)

# Add the standard library to the build
target_link_libraries(air_data_system
    pico_stdlib
    pico_multicore
    pico_bootrom
    pico_cxx_options
    pico_rand
    hardware_uart
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_dma
    hardware_irq
    hardware_timer
    hardware_pwm
    sh2
    sdcard
)

pico_add_extra_outputs(air_data_system)


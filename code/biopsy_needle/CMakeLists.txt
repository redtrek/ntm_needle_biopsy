#### CMAKE Config for NTM Aspiration Assisted Needle Biopsy Device
#### Author: Thomas Chang

cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/libs/pico-sdk)

# initialize pico-sdk directly
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

# include build functions from Pico-SDK
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

set(NAME biopsy_needle)

# project name and C standard
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# building for feather rp2040
#set(PICO_BOARD adafruit_feather_rp2040)


# intialize pico SDK
pico_sdk_init()

# optional flag
option(PCB "build for pcb or breadboard" 0)

# compile definitions
add_compile_definitions(PCB=${PCB})

# src files .h and .cpp
add_executable(${NAME}
    src/main.cpp
    src/ntm_helpers.cpp
    src/hw_config.c
    src/msc_disk.c
    src/usb_descriptors.c
    libs/SSD1306/ssd1306.c
    libs/SSD1306/ssd1306.h
    libs/SSD1306/font.h
    libs/FX29/fx29.h
    libs/FX29/fx29.c
)

add_subdirectory(libs/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src build)
add_subdirectory(libs/INA219)

# link libraries
target_link_libraries(${NAME} PRIVATE
    pico_stdlib
    hardware_clocks
    hardware_i2c
    hardware_adc
    hardware_pwm
    hardware_irq
    pico_multicore
    ina219
    no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
    tinyusb_additions
    tinyusb_board
    tinyusb_device
)

target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src/include
)

# usb/serial configurations
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)

# generate uf2 output files for flash
pico_add_extra_outputs(${NAME})


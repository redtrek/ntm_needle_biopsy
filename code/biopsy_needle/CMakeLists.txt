#### CMAKE Config for NTM Aspiration Assisted Needle Biopsy Device
#### Author: Thomas Chang

cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk directly
include(./libs/pico-sdk/pico_sdk_init.cmake)

# include build functions from Pico-SDK
include(./libs/pico-sdk/external/pico_sdk_import.cmake)

set(NAME biopsy_needle)

# project name and C standard
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# building for feather rp2040
set(PICO_BOARD adafruit_feather_rp2040)
#set(PICO_SDK_PATH libs/pico-sdk)

# intialize pico SDK
pico_sdk_init()

# src files .h and .cpp
add_executable(${NAME}
    src/main.cpp
    src/hw_config.c
    libs/SSD1306/ssd1306.c
    libs/SSD1306/ssd1306.h
    libs/SSD1306/font.h
)

add_subdirectory(libs/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src build)
add_subdirectory(libs/INA219_driver)

# link libraries
target_link_libraries(${NAME} PRIVATE
    pico_stdlib
    hardware_i2c
    hardware_adc
    hardware_pwm
    hardware_irq
    pico_multicore
    ina219
    no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
)

target_include_directories(${NAME} PRIVATE
    libs/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/include
)

# generate uf2 output files for flash
pico_add_extra_outputs(${NAME})

# enable usb
pico_enable_stdio_usb(${NAME} 1)

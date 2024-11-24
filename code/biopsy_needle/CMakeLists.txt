#### CMAKE Config for NTM Aspiration Assisted Needle Biopsy Device
#### Author: Thomas Chang

cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk directly
include(./libs/pico-sdk/pico_sdk_init.cmake)
include(./libs/pico-sdk/external/pico_sdk_import.cmake)

# project name and C standard
project(biopsy_needle C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# building for feather rp2040
set(PICO_BOARD adafruit_feather_rp2040)
#set(PICO_SDK_PATH libs/pico-sdk)

# intialize pico SDK
pico_sdk_init()

# src files .h and .cpp
add_executable(biopsy_needle
    src/main.cpp
)

# link libraries
target_link_libraries(biopsy_needle
    pico_stdlib
)

# generate uf2 output files for flash
pico_add_extra_outputs(biopsy_needle)

# enable usb
pico_enable_stdio_usb(biopsy_needle 1)
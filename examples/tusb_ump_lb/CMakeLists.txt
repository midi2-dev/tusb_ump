# tusb_ump_lb - canned USB MIDI 2.0 (UMP) loopback example
#
# Default target is a Raspberry Pi Pico (RP2040) board, but the firmware
# itself (src/main.c, src/usb_descriptors.c) has no RP2040-specific code
# beyond board bring-up in main.c. Porting to another pico-sdk-supported
# board is just a matter of changing PICO_BOARD below (or on the cmake
# command line). Porting to a non pico-sdk MCU means swapping this
# CMakeLists.txt / pico_sdk_import.cmake for that target's TinyUSB
# integration and keeping ../../ump_device.c, ../../ump_device.h and
# ../../ump.h as-is - they are a generic TinyUSB class driver.

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(tusb_ump_lb C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.0.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.0.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

pico_sdk_init()

add_executable(tusb_ump_lb
        src/main.c
        src/usb_descriptors.c
        ../../ump_device.cpp
        )

target_include_directories(tusb_ump_lb PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/src
        ../..
        )

target_link_libraries(tusb_ump_lb
        pico_stdlib
        pico_unique_id
        tinyusb_device
        )

# Logs (boot message, alt-setting changes, echoed UMP words) go out the
# board's default UART, not the USB MIDI 2.0 interface itself.
pico_enable_stdio_uart(tusb_ump_lb 1)
pico_enable_stdio_usb(tusb_ump_lb 0)

pico_add_extra_outputs(tusb_ump_lb)

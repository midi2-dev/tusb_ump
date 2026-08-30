# tusb_ump_uart_din_bridge - DIN MIDI 1.0 <-> USB MIDI 2.0 (UMP) bridge over
# a real RP2040 hardware UART peripheral.
#
# Default target is a Raspberry Pi Pico (RP2040) board, but nothing in
# src/main.c, src/usb_descriptors.c, or src/midi1_bytestream.c is
# RP2040-specific beyond board bring-up in main.c (which UART/pins to use,
# stdio_init_all()). Porting to another pico-sdk-supported board is just a
# matter of changing PICO_BOARD below (or on the cmake command line), and
# picking a UART instance/pins that don't collide with that board's own
# console UART. Porting to a non pico-sdk MCU means swapping this
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

project(tusb_ump_uart_din_bridge C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.0.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.0.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

pico_sdk_init()

add_executable(tusb_ump_uart_din_bridge
        src/main.c
        src/usb_descriptors.c
        src/midi1_bytestream.c
        ../../ump_device.cpp
        )

target_include_directories(tusb_ump_uart_din_bridge PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/src
        ../..
        )

target_link_libraries(tusb_ump_uart_din_bridge
        pico_stdlib
        pico_unique_id
        hardware_uart
        tinyusb_device
        )

# Logs (boot message, alt-setting changes) go out the board's default UART
# (UART0/GP0-1), not the DIN port (UART1/GP4-5 by default -- see main.c) or
# the USB MIDI 2.0 interface itself.
pico_enable_stdio_uart(tusb_ump_uart_din_bridge 1)
pico_enable_stdio_usb(tusb_ump_uart_din_bridge 0)

pico_add_extra_outputs(tusb_ump_uart_din_bridge)

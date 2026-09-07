# tusb_ump_host_demo - USB Host UMP bring-up / demo app
#
# Default target is a Raspberry Pi Pico (RP2040) board. main.cpp has no
# RP2040-specific code beyond board bring-up and the tinyusb_overrides/
# hcd_rp2040.c swap below (an RP2040 host-controller-driver bugfix, see that
# file's own header comment). Porting to another pico-sdk-supported board
# with a native USB host controller is a matter of changing PICO_BOARD
# below and dropping the hcd_rp2040.c override if that board's controller
# doesn't need it. Porting to a non pico-sdk MCU means swapping this
# CMakeLists.txt / pico_sdk_import.cmake for that target's TinyUSB host
# integration and keeping ../../ump_host.cpp, ../../ump_host.h and
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

project(tusb_ump_host_demo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.0.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.0.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

pico_sdk_init()

add_executable(tusb_ump_host_demo
        src/main.cpp
        ../../ump_host.cpp
        )

target_include_directories(tusb_ump_host_demo PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/src
        ../..
        )

target_link_libraries(tusb_ump_host_demo
        pico_stdlib
        pico_stdio
        tinyusb_host
        )

# Swap in the patched hcd_rp2040.c (see tinyusb_overrides/hcd_rp2040.c's own
# header comment: reports a genuine hardware RX_TIMEOUT condition as
# XFER_RESULT_TIMEOUT instead of silently hanging the shared control
# endpoint forever) in place of pico-sdk's stock copy, which
# tinyusb_host_base injects into every linking target via
# target_sources(... INTERFACE ...). Scoped to this app's target only - the
# shared pico-sdk install is never modified.
get_target_property(_tinyusb_host_base_srcs tinyusb_host_base INTERFACE_SOURCES)
list(FILTER _tinyusb_host_base_srcs EXCLUDE REGEX "portable/raspberrypi/rp2040/hcd_rp2040\\.c$")
set_target_properties(tinyusb_host_base PROPERTIES INTERFACE_SOURCES "${_tinyusb_host_base_srcs}")
target_sources(tusb_ump_host_demo PRIVATE tinyusb_overrides/hcd_rp2040.c)
# rp2040_usb.h is a private header that lives alongside the stock
# hcd_rp2040.c in the SDK (not on any public include path); our override
# needs it too.
target_include_directories(tusb_ump_host_demo PRIVATE
        ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/rp2040
        )

# Logs (device enumeration, GTB dumps, received UMP words) go out the
# board's default UART, not USB - the only USB port is occupied by host
# role itself.
pico_enable_stdio_uart(tusb_ump_host_demo 1)
pico_enable_stdio_usb(tusb_ump_host_demo 0)

pico_add_extra_outputs(tusb_ump_host_demo)

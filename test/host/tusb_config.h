// Minimal TinyUSB config for host-side unit testing of ump_device.cpp.
// Not a real MCU target -- only tusb_fifo.c and ump_device.cpp are
// compiled/linked, with the rest of the USB device stack stubbed out in
// stubs.c, so most of this only needs to satisfy header requirements.
#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#define CFG_TUSB_MCU              OPT_MCU_NONE
#define CFG_TUSB_OS               OPT_OS_NONE
#define BOARD_DEVICE_RHPORT_NUM   0
#define CFG_TUSB_RHPORT0_MODE     (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif
#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN
#endif

#define CFG_TUD_ENDPOINT0_SIZE    64

#define CFG_TUD_CDC               0
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            0
#define CFG_TUD_UMP               1

// Small on purpose: buffer-boundary behavior is exactly what these tests
// exercise, and a small FIFO makes it easy to feed precisely-sized bursts.
#define CFG_TUD_UMP_RX_BUFSIZE    64
#define CFG_TUD_UMP_TX_BUFSIZE    64

#ifdef __cplusplus
}
#endif

#endif

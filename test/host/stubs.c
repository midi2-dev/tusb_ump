// Minimal stand-ins for the TinyUSB device-stack entry points ump_device.cpp
// references (control transfer / endpoint management / xfer/itf-selection
// callbacks), so this test can link without a real USB device stack or
// hardware. None of these are exercised by the read-path tests here --
// they only satisfy the linker for umpd_open/umpd_control_xfer_cb/
// umpd_xfer_cb and _prep_out_transaction(), none of which the tests call
// except the latter (invoked at the end of every tud_ump_read_impl()).
#include <stdbool.h>
#include <stdint.h>

#include "device/usbd.h"
#include "ump_device.h"

bool usbd_edpt_claim(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  (void) ep_addr;
  return true;
}

bool usbd_edpt_release(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  (void) ep_addr;
  return true;
}

bool usbd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes)
{
  (void) rhport;
  (void) ep_addr;
  (void) buffer;
  (void) total_bytes;
  return true;
}

bool usbd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * desc_ep)
{
  (void) rhport;
  (void) desc_ep;
  return true;
}

bool tud_control_xfer(uint8_t rhport, tusb_control_request_t const * request, void* buffer, uint16_t len)
{
  (void) rhport;
  (void) request;
  (void) buffer;
  (void) len;
  return true;
}

bool tud_control_status(uint8_t rhport, tusb_control_request_t const * request)
{
  (void) rhport;
  (void) request;
  return true;
}

void tud_ump_rx_cb(uint8_t itf)
{
  (void) itf;
}

void tud_ump_set_itf_cb(uint8_t itf, uint8_t alt)
{
  (void) itf;
  (void) alt;
}

bool tud_ump_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void) rhport;
  (void) p_request;
  return false;
}

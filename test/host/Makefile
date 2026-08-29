# Host-side unit test build for ump_device.cpp's read path. No cross
# toolchain or hardware needed -- links the real ump_device.cpp against the
# real (vendored) TinyUSB tusb_fifo.c, with the small stubs.c standing in
# for the rest of the USB device stack.
#
# TUSB_ROOT can be overridden to point at any TinyUSB checkout/vendor copy
# whose src/ layout matches upstream, e.g.:
#   make TUSB_ROOT=/path/to/pico-sdk/lib/tinyusb
TUSB_ROOT ?= $(HOME)/.pico-sdk/sdk/2.3.0/lib/tinyusb
TUSB_SRC  := $(TUSB_ROOT)/src

CC  := cc
CXX := c++

CPPFLAGS := -I. -I../.. -I$(TUSB_SRC) -DUMP_DEVICE_UNIT_TEST
CFLAGS   := -std=c11   -Wall -Wextra -g
CXXFLAGS := -std=c++17 -Wall -Wextra -g

OBJS := test_ump_device.o stubs.o tusb_fifo.o ump_device.o

test_ump_device: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

test_ump_device.o: test_ump_device.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

stubs.o: stubs.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

tusb_fifo.o: $(TUSB_SRC)/common/tusb_fifo.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

ump_device.o: ../../ump_device.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

.PHONY: check clean
check: test_ump_device
	./test_ump_device

clean:
	rm -f $(OBJS) test_ump_device

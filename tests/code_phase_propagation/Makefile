BINARY = code_phase_propagation_test

OBJS = code_phase_propagation_test.o \
       ../../src/debug.o \
       ../../src/swift_nap_io.o \
       ../../src/acq.o \
       ../../src/track.o \
       ../../src/nav_msg.o \
       ../../src/hw/max2769.o \
       ../../src/hw/m25_flash.o \
       ../../src/hw/leds.o \
       ../../src/hw/spi.o \
       ../../src/hw/usart.o \

CFLAGS += -I../../src/

ifdef PRN
	CFLAGS += -DPRN=$(PRN)
endif

SWIFTNAV_ROOT ?= ../..

include ../../stm32/Makefile.include


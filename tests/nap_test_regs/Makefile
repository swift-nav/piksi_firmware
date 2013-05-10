BINARY = nap_test_regs_test

OBJS = nap_test_regs_test.o \
       ../../src/debug.o \
       ../../src/settings.o \
       ../../src/error.o \
       ../../src/swift_nap_io.o \
       ../../src/cw.o \
       ../../src/acq.o \
       ../../src/track.o \
       ../../src/hw/max2769.o \
       ../../src/hw/m25_flash.o \
       ../../src/hw/leds.o \
       ../../src/hw/spi.o \
       ../../src/hw/usart.o \
       ../../src/hw/usart_tx.o \
       ../../src/hw/usart_rx.o \

CFLAGS += -I../../src/

SWIFTNAV_ROOT ?= ../..

include ../../stm32/Makefile.include


BINARY = leds_test

OBJS = leds_test.o \
       ../../src/error.o \
       ../../src/hw/leds.o \

CFLAGS += -I../../src/

LDFLAGS += -Ttext=0x08010000

SWIFTNAV_ROOT ?= ../..

include ../../stm32/Makefile.include


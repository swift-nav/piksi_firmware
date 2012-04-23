BINARY = debug_stress_test

OBJS = debug_stress_test.o \
       ../../src/debug.o \
       ../../src/error.o \
       ../../src/hw/leds.o \
       ../../src/hw/usart.o \
       ../../src/hw/usart_tx.o \
       ../../src/hw/usart_rx.o \

CFLAGS += -I../../src/

SWIFTNAV_ROOT ?= ../..

include ../../stm32/Makefile.include


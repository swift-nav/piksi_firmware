BINARY = bootloader

OBJS = bootloader.o \
       ../src/debug.o \
       ../src/settings.o \
       ../src/error.o \
       ../src/hw/usart.o \
       ../src/hw/usart_rx.o \
       ../src/hw/usart_tx.o \
       ../src/hw/leds.o \
       ../src/hw/stm_flash.o \

CFLAGS += -I../src/

SWIFTNAV_ROOT ?= ..

LDSCRIPT ?= $(SWIFTNAV_ROOT)/bootloader/bootloader.ld

include $(SWIFTNAV_ROOT)/stm32/Makefile.include

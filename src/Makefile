BINARY = swiftnav-firmware

SWIFTLIB_OBJS = ../swiftlib/linear_algebra.o \
                ../swiftlib/coord_system.o \
                ../swiftlib/pvt.o \

OBJS = main.o \
       debug.o \
       swift_nap_io.o \
       track.o \
       nav_msg.o \
       acq.o \
       manage.o \
       ephemeris.o \
       hw/m25_flash.o \
       hw/max2769.o \
       hw/leds.o \
       hw/spi.o \
       hw/usart.o \
       $(SWIFTLIB_OBJS)

include ../stm32/Makefile.include


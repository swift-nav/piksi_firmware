##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
## Copyright (C) 2011-2014 Swift Navigation Inc <info@swift-nav.com>
## Contact: Fergus Noble <fergus@swift-nav.com>
##          Colin Beighley <colin@swift-nav.com>
##
## Derived from libopenstm32 example Makefile.include
##
## This source is subject to the license found in the file 'LICENSE' which must
## be be distributed together with this source. All other rights reserved.
##
## THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
## EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
## WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
##

BINARY = bootloader

LDSCRIPT = bootloader.ld

SWIFTNAV_ROOT ?= ..
PREFIX ?= arm-none-eabi
GIT_VERSION := $(shell $(SWIFTNAV_ROOT)/scripts/git-check.sh)
# PREFIX ?= arm-elf
CC = $(PREFIX)-gcc
LD = $(PREFIX)-gcc
OBJCOPY = $(PREFIX)-objcopy
OBJDUMP = $(PREFIX)-objdump

# Search these directories for .c files
vpath %.c $(SWIFTNAV_ROOT)/src
vpath %.c $(SWIFTNAV_ROOT)/src/board
vpath %.c $(SWIFTNAV_ROOT)/src/board/nap
vpath %.c $(SWIFTNAV_ROOT)/src/peripherals

OBJS = \
  bootloader.o \
  sbp.o \
  leds.o \
  stm_flash.o \
  usart.o \
  usart_tx.o \
  usart_rx.o \
  error.o \
  settings.o \
  spi.o \
  m25_flash.o \
  nap_common.o \
  flash_callbacks.o \

CFLAGS += -Os -Wall -Wextra -Werror -std=gnu99 \
			    -fdata-sections -ffunction-sections \
			    -fno-common -MD -DSTM32F4 -DGIT_VERSION="\"$(GIT_VERSION)\"" \
			    -mcpu=cortex-m4 -march=armv7e-m -mthumb \
			    -mfloat-abi=hard -mfpu=fpv4-sp-d16

CFLAGS += -I$(SWIFTNAV_ROOT)/src -I../libopencm3/include

LDSCRIPT ?= $(SWIFTNAV_ROOT)/stm32/swiftnav.ld
LDFLAGS += -lc -lnosys -T$(LDSCRIPT) -nostartfiles \
					 -Wl,-wrap,printf -Wl,--gc-sections \
					 -mcpu=cortex-m4 -march=armv7e-m -mthumb \
					 -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
					 -lopencm3_stm32f4 -lswiftnav-static-cortex-m4 -lm -L../libopencm3/lib

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
NULL := 2>/dev/null
else
LDFLAGS += -Wl,--print-gc-sections
endif

.SUFFIXES: .elf .bin .hex .srec .list .images
.SECONDEXPANSION:
.SECONDARY:

all: images

images: $(BINARY).images
flash: $(BINARY).flash

%.images: %.bin %.hex %.srec %.list
	@echo "*** $* images generated ***"

%.bin: %.elf
	@printf "  OBJCOPY $(*).bin\n"
	$(Q)$(OBJCOPY) -Obinary $(*).elf $(*).bin

%.hex: %.elf
	@printf "  OBJCOPY $(*).hex\n"
	$(Q)$(OBJCOPY) -Oihex $(*).elf $(*).hex

%.srec: %.elf
	@printf "  OBJCOPY $(*).srec\n"
	$(Q)$(OBJCOPY) -Osrec $(*).elf $(*).srec

%.list: %.elf
	@printf "  OBJDUMP $(*).list\n"
	$(Q)$(OBJDUMP) -S $(*).elf > $(*).list

%.elf: $(OBJS)
	@printf "  LD      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(LD) -o $(*).elf $(OBJS) -Xlinker -Map=$(*).map $(LDFLAGS)

%.o: %.c Makefile
	@printf "  CC      bootloader/$(@)\n"
	$(Q)$(CC) $(CFLAGS) -o ./$@ -c $<

clean:
	$(Q)rm -f $(OBJS)
	$(Q)rm -f $(OBJS:.o=.d)
	$(Q)rm -f *.elf
	$(Q)rm -f *.bin
	$(Q)rm -f *.hex
	$(Q)rm -f *.srec
	$(Q)rm -f *.list
	$(Q)rm -f *.map

.PHONY: images clean

-include $(OBJS:.o=.d)


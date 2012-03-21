##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2012 Fergus Noble <fergusnoble@gmail.com>
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.
##

LIBNAME		= libswiftnav-arm

PREFIX	?= arm-none-eabi
CC		= $(PREFIX)-gcc
AR		= $(PREFIX)-ar
CFLAGS += -O3 -g -Wall -Wextra -Werror \
		   -mcpu=cortex-m3 -mthumb -Wstrict-prototypes \
		   -ffunction-sections -fdata-sections -MD \
		   -fno-common -msoft-float \
		   -DGIT_VERSION="\"$(GIT_VERSION)\"" -std=gnu99
# ARFLAGS	= rcsv
ARFLAGS	= rcs
OBJS = linear_algebra.o \
			 coord_system.o \
			 pvt.o \
			 track.o \
			 nav_msg.o \
			 ephemeris.o \
			 tropo.o \
			 ca_codes.o \

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
endif

all: $(LIBNAME).a

$(LIBNAME).a: $(OBJS)
	@printf "  AR      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(AR) $(ARFLAGS) $@ $^

%.o: %.c
	@printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

clean:
	$(Q)rm -f *.o *.d
	$(Q)rm -f $(LIBNAME).a

.PHONY: clean

-include $(OBJS:.o=.d)


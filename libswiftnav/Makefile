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

LIBNAME		= libswiftnav

# Do not print "Entering directory ...".
MAKEFLAGS += --no-print-directory

CC = gcc
COMMON_CFLAGS += -O3 -g -Wall -Wextra -Werror \
		   -ffunction-sections -fdata-sections -MD \
		   -Wstrict-prototypes -fno-common \
		   -DGIT_VERSION="\"$(GIT_VERSION)\"" -std=gnu99
CFLAGS += $(COMMON_CFLAGS) -fprofile-arcs -ftest-coverage
ARFLAGS	= rcs
SRCS = linear_algebra.c \
			 coord_system.c \
			 pvt.c \
			 track.c \
			 nav_msg.c \
			 ephemeris.c \
			 tropo.c \
			 prns.c
OBJS = $(SRCS:.c=.o)

ARM_PREFIX	?= arm-none-eabi
ARM_CC		= $(ARM_PREFIX)-gcc
ARM_AR		= $(ARM_PREFIX)-ar
ARM_CFLAGS += $(COMMON_CFLAGS) -mcpu=cortex-m3 -mthumb -msoft-float
ARM_OBJS = $(SRCS:.c=.arm.o)

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
endif

all: $(LIBNAME)-x86.a $(LIBNAME)-arm.a check

$(LIBNAME)-arm.a: $(ARM_OBJS)
	@printf "  ARM AR   $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(ARM_AR) $(ARFLAGS) $@ $^

$(LIBNAME)-x86.a: $(OBJS)
	@printf "  x86 AR   $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(AR) $(ARFLAGS) $@ $^

%.arm.o: %.c
	@printf "  ARM CC   $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(ARM_CC) $(ARM_CFLAGS) -o $@ -c $<

%.o: %.c
	@printf "  x86 CC   $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

check: $(LIBNAME)-x86.a
	@printf "  CHECK    libswiftnav\n"; \
	$(MAKE) -C tests $(MAKEFLAGS) check

lcov:
	mkdir -p lcov
	lcov -d ./ -z
	make check
	lcov -d ./ -c --output-file lcov/coverage.info
	cd lcov && genhtml --legend coverage.info

clean:
	$(Q)rm -f *.arm.o *.o *.d *.a
	$(Q)rm -f *.gcda *.gcno
	$(Q)rm -rf lcov/
	@printf "  CLEAN    tests\n"; \
	$(MAKE) -C tests $(MAKEFLAGS) clean

# Program to run for Google CPP Lint code
LINT = $(shell command -v cpplint || command -v cpplint.py)

# A target for emacs' flymake-mode!
check-syntax:
	-$(Q)$(CC) $(CFLAGS) -fsyntax-only $(CHK_SOURCES)
	-$(LINT) $(CHK_SOURCES) 2>&1 | grep -v "C-style cast"

lint: 
	$(MAKE) check-syntax CHK_SOURCES="$(SRCS)"

.PHONY: clean check lcov check-syntax lint

-include $(SRCS:.c=.d)


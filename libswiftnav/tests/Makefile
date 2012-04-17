
CC = gcc
CFLAGS += -O0 -g -Wall -Wextra -Werror \
			 -ffunction-sections -fdata-sections -MD \
			 -Wstrict-prototypes -fno-common \
			 -DGIT_VERSION="\"$(GIT_VERSION)\"" -std=gnu99 \
			 -I../
LDFLAGS += -lm -L../ -fprofile-arcs

SRCS = check_main.c \
			 check_utils.c \
			 check_coord_system.c \
			 check_linear_algebra.c \

# Program to run for Google CPP Lint code
LINT = $(shell command -v cpplint || command -v cpplint.py)

OBJS = $(SRCS:.c=.o)

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
endif

%.o: %.c
	@printf "    CC     $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

check_libswiftnav: $(OBJS) ../libswiftnav-x86.a
	@printf "    LD     $@\n"
	$(Q)$(CC) -o $@ $(OBJS) -lcheck -lswiftnav-x86 $(LDFLAGS)

check: check_libswiftnav
	@echo "    RUNNING UNIT TESTS"
	@echo "-------------------------------------------------------"
	@./check_libswiftnav
	@echo "-------------------------------------------------------"

clean:
	$(Q)rm -f *.o *.d check_libswiftnav

check-syntax:
	-$(Q)$(CC) $(CFLAGS) -fsyntax-only $(CHK_SOURCES)
	-$(LINT) $(CHK_SOURCES) 2>&1 | grep -v "C-style cast"

.PHONY: clean check check-syntax

-include $(SRCS:.c=.d)



CC = gcc
CFLAGS += -O0 -g -Wall -Wextra -Werror \
			 -ffunction-sections -fdata-sections -MD \
			 -Wstrict-prototypes -fno-common \
			 -DGIT_VERSION="\"$(GIT_VERSION)\"" -std=gnu99 \
			 -I../ -I/opt/local/include
LDFLAGS += -lm -L../ -L/opt/local/lib -fprofile-arcs

SRCS = check_main.c \
			 check_utils.c \
			 check_coord_system.c \
			 check_linear_algebra.c \


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

.PHONY: clean check

-include $(SRCS:.c=.d)


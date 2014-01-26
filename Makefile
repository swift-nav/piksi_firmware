SWIFTNAV_ROOT := $(shell pwd)
MAKEFLAGS += SWIFTNAV_ROOT=$(SWIFTNAV_ROOT)

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
# Do not print "Entering directory ...".
MAKEFLAGS += --no-print-directory
endif

ifneq (,$(findstring W32,$(shell uname)))
	CMAKEFLAGS += -G "MSYS Makefiles"
endif

ifndef PRN
	MAKEFLAGS += $(warning PRN not defined, using default PRN (22) for tests, specify the PRN with 'make PRN=22')PRN=22
else
	MAKEFLAGS += PRN=$(PRN)
endif

.PHONY: all tests firmware bootloader docs libopencm3 libswiftnav

all: firmware bootloader tests

firmware: libopencm3 libswiftnav
	@printf "BUILD   src\n"; \
	$(MAKE) -C src $(MAKEFLAGS)

bootloader:
	@printf "BUILD   bootloader\n"; \
	$(MAKE) -C bootloader $(MAKEFLAGS)

tests:
	$(Q)for i in tests/*; do \
		if [ -d $$i ]; then \
			printf "BUILD   $$i\n"; \
			$(MAKE) -C $$i $(MAKEFLAGS) || exit $?; \
		fi; \
	done

libopencm3:
	@printf "BUILD   libopencm3\n"; \
	$(MAKE) -C libopencm3 $(MAKEFLAGS)

libswiftnav:
	@printf "BUILD   libswiftnav\n"; \
	mkdir -p libswiftnav/build; cd libswiftnav/build; \
	cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-gcc-arm-embedded.cmake $(CMAKEFLAGS) ../
	
	$(MAKE) -C libswiftnav/build $(MAKEFLAGS)
clean:
	@printf "CLEAN   src\n"; \
	$(MAKE) -C src $(MAKEFLAGS) clean
	@printf "CLEAN   bootloader\n"; \
	$(MAKE) -C bootloader $(MAKEFLAGS) clean
	@printf "CLEAN   libopencm3\n"; \
	$(MAKE) -C libopencm3 $(MAKEFLAGS) clean
	@printf "CLEAN   libswiftnav\n"; \
	$(RM) -rf libswiftnav/build
	$(Q)for i in tests/*; do \
		if [ -d $$i ]; then \
			printf "CLEAN   $$i\n"; \
			$(MAKE) -C $$i $(MAKEFLAGS) clean || exit $?; \
		fi; \
	done

docs:
	$(MAKE) -C docs/diagrams
	doxygen docs/Doxyfile


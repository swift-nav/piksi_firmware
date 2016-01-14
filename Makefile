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

.PHONY: all tests firmware docs hitl .FORCE

all: firmware # tests

firmware: libopencm3/lib/libopencm3_stm32f4.a libsbp/c/build/src/libsbp-static.a libswiftnav/build/src/libswiftnav-static.a
	@printf "BUILD   src\n"; \
	$(MAKE) -r -C src $(MAKEFLAGS)

tests:
	$(Q)for i in tests/*; do \
		if [ -d $$i ]; then \
			printf "BUILD   $$i\n"; \
			$(MAKE) -r -C $$i $(MAKEFLAGS) || exit $?; \
		fi; \
	done

libopencm3/lib/libopencm3_stm32f4.a:
	@printf "BUILD   libopencm3\n"; \
	$(MAKE) -C libopencm3 $(MAKEFLAGS) lib/stm32/f4

libsbp/c/build/src/libsbp-static.a:
	@printf "BUILD   libsbp\n"; \
	mkdir -p libsbp/c/build; cd libsbp/c/build; \
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-gcc-arm-embedded.cmake $(CMAKEFLAGS) ../
	$(MAKE) -C libsbp/c/build $(MAKEFLAGS)

libswiftnav/build/src/libswiftnav-static.a: .FORCE
	@printf "BUILD   libswiftnav\n"; \
	mkdir -p libswiftnav/build; cd libswiftnav/build; \
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-gcc-arm-embedded.cmake $(CMAKEFLAGS) ../
	$(MAKE) -C libswiftnav/build $(MAKEFLAGS)

clean:
	@printf "CLEAN   src\n"; \
	$(MAKE) -C src $(MAKEFLAGS) clean
	@printf "CLEAN   libopencm3\n"; \
	$(MAKE) -C libopencm3 $(MAKEFLAGS) clean
	@printf "CLEAN   libsbp\n"; \
	$(RM) -rf libsbp/c/build
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

hitl: firmware
	@printf "Running default HITL jobs\n"
	if cd build/hitl_tools; then \
		git pull; \
	else \
		git clone git@github.com:swift-nav/hitl_tools.git build/hitl_tools --depth 1; \
	fi
	bash build/hitl_tools/make_hitl.sh

.FORCE:

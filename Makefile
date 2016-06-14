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

ifeq ($(PIKSI_HW),)
  PIKSI_HW=v2
endif

# Imported source files and paths
CHIBIOS = ./ChibiOS
include $(SWIFTNAV_ROOT)/src/board/$(PIKSI_HW)/Makefile.inc

MAKEFLAGS += PIKSI_HW=$(PIKSI_HW)

ifeq ($(PIKSI_HW),v2)
	CMAKEFLAGS += -DCMAKE_SYSTEM_PROCESSOR=cortex-m4 -DMAX_CHANNELS=$(MAX_CHANNELS)
endif

ifeq ($(PIKSI_HW),v3)
	CMAKEFLAGS += -DCMAKE_SYSTEM_PROCESSOR=cortex-a9 -DMAX_CHANNELS=$(MAX_CHANNELS)
endif

.PHONY: all tests firmware docs hitl_setup hitl hitlv3 .FORCE

all: firmware # tests

firmware: libsbp/c/build/src/libsbp-static.a libswiftnav/build/src/libswiftnav-static.a
	@printf "BUILD   src\n"; \
	$(MAKE) -r -C src $(MAKEFLAGS)

tests:
	$(Q)for i in tests/*; do \
		if [ -d $$i ]; then \
			printf "BUILD   $$i\n"; \
			$(MAKE) -r -C $$i $(MAKEFLAGS) || exit $?; \
		fi; \
	done

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

hitl_setup: firmware
	# Usage:
	# `make hitl` will run the default "quick" test plan (1 capture job)
	# Optionally specify a desired test plan:
	# `make hitl TEST_PLAN=merge` will run the "merge" test plan (10 capture jobs)
	#
	# First, this script will pull or clone the hitl_tools repo.
	if cd build/hitl_tools; then \
		git pull; \
	else \
		git clone git@github.com:swift-nav/hitl_tools.git build/hitl_tools --depth 1; \
	fi

hitl: hitl_setup
	TEST_PLAN=$(TEST_PLAN) TEST_CONFIG=$(TEST_CONFIG) bash build/hitl_tools/make_hitl.sh


hitlv3: hitl_setup
	TEST_PLAN=$(TEST_PLAN) TEST_CONFIG=v3_config bash build/hitl_tools/make_hitl.sh

.FORCE:

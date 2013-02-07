SWIFTNAV_ROOT := $(shell pwd)
MAKEFLAGS += SWIFTNAV_ROOT=$(SWIFTNAV_ROOT)

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
# Do not print "Entering directory ...".
MAKEFLAGS += --no-print-directory
endif

ifndef PRN
	MAKEFLAGS += $(warning PRN not defined, using default PRN (22) for tests, specify the PRN with 'make PRN=22')PRN=22
else
	MAKEFLAGS += PRN=$(PRN)
endif

.PHONY: all tests firmware docs libswiftnav

all: libswiftnav firmware tests

libswiftnav:
	@printf "BUILD   libswiftnav\n"; \
	$(MAKE) -C libswiftnav $(MAKEFLAGS)

firmware:
	@printf "BUILD   src\n"; \
	$(MAKE) -C src $(MAKEFLAGS)

tests: libswiftnav
	$(Q)for i in tests/*; do \
		if [ -d $$i ]; then \
			printf "BUILD   $$i\n"; \
			$(MAKE) -C $$i $(MAKEFLAGS) || exit $?; \
		fi; \
	done

clean:
	@printf "CLEAN   libswiftnav\n"; \
	$(MAKE) -C libswiftnav $(MAKEFLAGS) clean
	@printf "CLEAN   src\n"; \
	$(MAKE) -C src $(MAKEFLAGS) clean
	$(Q)for i in tests/*; do \
		if [ -d $$i ]; then \
			printf "CLEAN   $$i\n"; \
			$(MAKE) -C $$i $(MAKEFLAGS) clean || exit $?; \
		fi; \
	done

docs:
	$(MAKE) -C docs/diagrams
	doxygen docs/Doxyfile


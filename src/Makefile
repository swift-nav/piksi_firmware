##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

ifeq ($(PIKSI_HW),)
  PIKSI_HW=v2
endif

ifeq ($(SWIFTNAV_ROOT),)
  SWIFTNAV_ROOT = ..
endif

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O2 -ggdb3 -fomit-frame-pointer -falign-functions=16
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT =
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT =
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

ifeq ($(BUILDDIR),)
  BUILDDIR = $(SWIFTNAV_ROOT)/build
endif

#
# Build global options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = piksi_firmware

# Imported source files and paths
CHIBIOS = ../ChibiOS

include $(SWIFTNAV_ROOT)/src/board/$(PIKSI_HW)/Makefile.inc

include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
include $(CHIBIOS)/os/rt/rt.mk

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC := $(STARTUPSRC) \
        $(PORTSRC) \
        $(KERNSRC) \
        $(HALSRC) \
        $(OSALSRC) \
        $(PLATFORMSRC) \
        $(BOARDSRC) \
        $(SWIFTNAV_ROOT)/src/peripherals/3drradio.o \
        $(SWIFTNAV_ROOT)/src/peripherals/leds.o \
        $(SWIFTNAV_ROOT)/src/peripherals/usart.o \
        $(SWIFTNAV_ROOT)/src/peripherals/usart_chat.o \
        $(SWIFTNAV_ROOT)/src/cfs/cfs-coffee.o \
        $(SWIFTNAV_ROOT)/src/minIni/minIni.o \
        $(SWIFTNAV_ROOT)/src/minIni/minGlue.o \
        $(SWIFTNAV_ROOT)/src/sbp.o \
        $(SWIFTNAV_ROOT)/src/sbp_fileio.o \
        $(SWIFTNAV_ROOT)/src/sbp_utils.o \
        $(SWIFTNAV_ROOT)/src/track.o \
        $(SWIFTNAV_ROOT)/src/track_internal.o \
        $(SWIFTNAV_ROOT)/src/track_api.o \
        $(SWIFTNAV_ROOT)/src/manage.o \
        $(SWIFTNAV_ROOT)/src/settings.o \
        $(SWIFTNAV_ROOT)/src/timing.o \
        $(SWIFTNAV_ROOT)/src/ext_events.o \
        $(SWIFTNAV_ROOT)/src/position.o \
        $(SWIFTNAV_ROOT)/src/solution.o \
        $(SWIFTNAV_ROOT)/src/base_obs.o \
        $(SWIFTNAV_ROOT)/src/simulator.o \
        $(SWIFTNAV_ROOT)/src/simulator_data.o \
        $(SWIFTNAV_ROOT)/src/nmea.o \
        $(SWIFTNAV_ROOT)/src/system_monitor.o \
        $(SWIFTNAV_ROOT)/src/ephemeris.o \
        $(SWIFTNAV_ROOT)/src/pps.o \
        $(SWIFTNAV_ROOT)/src/decode.o \
        $(SWIFTNAV_ROOT)/src/signal.o \
        $(SWIFTNAV_ROOT)/src/l2c_capb.o \
        $(SWIFTNAV_ROOT)/src/sid_set.o \
        main.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC =

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(STARTUPASM) $(PORTASM) $(OSALASM) $(BOARDASM)

INCDIR = $(STARTUPINC) $(PORTINC) $(KERNINC) \
         $(HALINC) $(OSALINC) $(PLATFORMINC) $(BOARDINC) \
         $(CHIBIOS)/os/various \
         $(SWIFTNAV_ROOT)/libsbp/c/include \
         $(SWIFTNAV_ROOT)/libswiftnav/include \
         $(SWIFTNAV_ROOT)/src \
         $(SWIFTNAV_ROOT)/src/board \
         $(SWIFTNAV_ROOT)/libswiftnav/libfec/include \

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# Define C warning options here
CWARN = -Wall -Wextra -Werror -std=gnu99

# Define C++ warning options here
CPPWARN = -Wall -Wextra -Werror

#
# Compiler settings
##############################################################################

##############################################################################
# Start of default section
#

# List all default C defines here, like -D_DEBUG=1
GIT_VERSION := $(shell git describe --dirty)
DDEFS = -DGIT_VERSION="\"$(GIT_VERSION)\""

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List all default directories to look for include files here
DINCDIR =

# List the default directory to look for the libraries here
DLIBDIR = $(SWIFTNAV_ROOT)/libsbp/c/build/src \
          $(SWIFTNAV_ROOT)/libswiftnav/build/src \
          $(SWIFTNAV_ROOT)/libswiftnav/build/CBLAS/src \
          $(SWIFTNAV_ROOT)/libswiftnav/build/clapack-3.2.1-CMAKE/BLAS/SRC \
          $(SWIFTNAV_ROOT)/libswiftnav/build/clapack-3.2.1-CMAKE/SRC \
          $(SWIFTNAV_ROOT)/libswiftnav/build/clapack-3.2.1-CMAKE/F2CLIBS/libf2c \
          $(SWIFTNAV_ROOT)/libswiftnav/build/libfec/src \
          $(SWIFTNAV_ROOT)/libswiftnav/build/src

# List all default libraries here
DLIBS = -lsbp-static -lswiftnav-static \
        -llapack -lcblas -lblas \
        -lf2c -lm -lc -lnosys -lfec

#
# End of default section
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

#
# End of user defines
##############################################################################

.DEFAULT_GOAL = all
include $(SWIFTNAV_ROOT)/ext/Makefile.include

include $(RULESPATH)/rules.mk


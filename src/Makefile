##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

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
# Architecture or project specific options
#

# Enables the use of FPU on Cortex-M4.
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = piksi_firmware

# Imported source files and paths
CHIBIOS = ../ChibiOS-RT
include $(CHIBIOS)/os/ports/GCC/ARMCMx/STM32F4xx/port.mk
include $(CHIBIOS)/os/kernel/kernel.mk

# Define linker script file here
LDSCRIPT= STM32F405xG.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(PORTSRC) \
       $(KERNSRC) \
       $(SWIFTNAV_ROOT)/src/board/nap/nap_common.o \
       $(SWIFTNAV_ROOT)/src/board/nap/nap_exti.o \
       $(SWIFTNAV_ROOT)/src/board/nap/nap_conf.o \
       $(SWIFTNAV_ROOT)/src/board/nap/acq_channel.o \
       $(SWIFTNAV_ROOT)/src/board/nap/track_channel.o \
       $(SWIFTNAV_ROOT)/src/board/nap/cw_channel.o \
       $(SWIFTNAV_ROOT)/src/board/m25_flash.o \
       $(SWIFTNAV_ROOT)/src/board/max2769.o \
       $(SWIFTNAV_ROOT)/src/board/leds.o \
       $(SWIFTNAV_ROOT)/src/peripherals/stm_flash.o \
       $(SWIFTNAV_ROOT)/src/peripherals/spi.o \
       $(SWIFTNAV_ROOT)/src/peripherals/usart.o \
       $(SWIFTNAV_ROOT)/src/peripherals/usart_tx.o \
       $(SWIFTNAV_ROOT)/src/peripherals/usart_rx.o \
       $(SWIFTNAV_ROOT)/src/cfs/cfs-coffee.o \
       $(SWIFTNAV_ROOT)/src/cfs/cfs-coffee-arch.o \
       $(SWIFTNAV_ROOT)/src/init.o \
       $(SWIFTNAV_ROOT)/src/sbp.o \
       $(SWIFTNAV_ROOT)/src/error.o \
       $(SWIFTNAV_ROOT)/src/cw.o \
       $(SWIFTNAV_ROOT)/src/track.o \
       $(SWIFTNAV_ROOT)/src/acq.o \
       $(SWIFTNAV_ROOT)/src/manage.o \
       $(SWIFTNAV_ROOT)/src/settings.o \
       $(SWIFTNAV_ROOT)/src/timing.o \
       $(SWIFTNAV_ROOT)/src/position.o \
       $(SWIFTNAV_ROOT)/src/solution.o \
       $(SWIFTNAV_ROOT)/src/simulator.o \
       $(SWIFTNAV_ROOT)/src/nmea.o \
       $(SWIFTNAV_ROOT)/src/system_monitor.o \
       $(SWIFTNAV_ROOT)/src/flash_callbacks.o \
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
ASMSRC = $(PORTASM)

INCDIR = $(PORTINC) $(KERNINC) $(CHIBIOS)/os/various \
         $(SWIFTNAV_ROOT)/libswiftnav/include \
         $(SWIFTNAV_ROOT)/src \
         $(SWIFTNAV_ROOT)/libopencm3/include

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

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

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -std=gnu99

# Define C++ warning options here
CPPWARN = -Wall -Wextra

#
# Compiler settings
##############################################################################

##############################################################################
# Start of default section
#

# List all default C defines here, like -D_DEBUG=1
GIT_VERSION := $(shell $(SWIFTNAV_ROOT)/scripts/git-check.sh)
DDEFS = -DSTM32F4 -DGIT_VERSION="\"$(GIT_VERSION)\""

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List all default directories to look for include files here
DINCDIR =

# List the default directory to look for the libraries here
DLIBDIR = $(SWIFTNAV_ROOT)/libopencm3/lib \
          $(SWIFTNAV_ROOT)/libswiftnav/build/src

# List all default libraries here
DLIBS = -lopencm3_stm32f4 -lswiftnav-static -lc -lm -lnosys

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

include $(SWIFTNAV_ROOT)/ChibiOS_rules.mk


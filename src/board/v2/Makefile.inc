##############################################################################
# Architecture or project specific options
#

# Enables the use of FPU on Cortex-M4.
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x738
endif

# Max tracking channels.
ifeq ($(MAX_CHANNELS),)
  MAX_CHANNELS = 11
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk
include $(CHIBIOS)/os/rt/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F4xx/platform.mk

BOARDDIR := $(SWIFTNAV_ROOT)/src/board/v2
BOARDINC := $(BOARDDIR) $(SWIFTNAV_ROOT)/src/board/nap

BOARDSRC := \
        $(BOARDDIR)/board.o \
        $(BOARDDIR)/cfs-coffee-arch.o \
        $(BOARDDIR)/error.o \
        $(BOARDDIR)/init.o \
        $(BOARDDIR)/acq.o \
        $(BOARDDIR)/nap/nap_common.o \
        $(BOARDDIR)/nap/nap_exti.o \
        $(BOARDDIR)/nap/nap_conf.o \
        $(BOARDDIR)/nap/nap_acq.o \
        $(BOARDDIR)/nap/acq_channel.o \
        $(BOARDDIR)/nap/track_channel.o \
        $(BOARDDIR)/nap/track_regs.o \
        $(BOARDDIR)/m25_flash.o \
        $(BOARDDIR)/max2769.o \
        $(BOARDDIR)/usart_support.o \
        $(BOARDDIR)/platform_signal.o \
        $(SWIFTNAV_ROOT)/src/peripherals/stm_flash.o \
        $(SWIFTNAV_ROOT)/src/peripherals/spi_wrapper.o \
        $(SWIFTNAV_ROOT)/src/peripherals/random.o \
        $(SWIFTNAV_ROOT)/src/peripherals/watchdog.o \
        $(SWIFTNAV_ROOT)/src/track/track_gps_l1ca.o \
        $(SWIFTNAV_ROOT)/src/decode/decode_gps_l1ca.o \

# Define linker script file here
LDSCRIPT= $(BOARDDIR)/STM32F405xG.ld

RULESPATH = $(BOARDDIR)

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

# THUMB-specific options here
TOPT += -mthumb -DTHUMB

#
# Compiler settings
##############################################################################


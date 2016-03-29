
USE_OPT += -mfpu=neon

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the ARM System/User stack. This
# stack is the stack used by the main() thread.
ifeq ($(USE_SYSTEM_STACKSIZE),)
  USE_SYSTEM_STACKSIZE = 0x400
endif

# Stack size to the allocated to the ARM IRQ stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_IRQ_STACKSIZE = 0x400
endif

# Stack size to the allocated to the ARM FIQ stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_FIQ_STACKSIZE),)
  USE_FIQ_STACKSIZE = 64
endif

# Stack size to the allocated to the ARM Supervisor stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_SUPERVISOR_STACKSIZE),)
  USE_SUPERVISOR_STACKSIZE = 8
endif

# Stack size to the allocated to the ARM Undefined stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_UND_STACKSIZE),)
  USE_UND_STACKSIZE = 8
endif

# Stack size to the allocated to the ARM Abort stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_ABT_STACKSIZE),)
  USE_ABT_STACKSIZE = 8
endif

# Enables the use of FPU.
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

include $(CHIBIOS)/os/common/ports/ARM/compilers/GCC/mk/startup_zynq7000.mk
include $(CHIBIOS)/os/hal/ports/ZYNQ7000/platform.mk
include $(CHIBIOS)/os/rt/ports/ARM/compilers/GCC/mk/port_generic.mk

BOARDDIR := $(SWIFTNAV_ROOT)/src/board/v3
BOARDINC := $(BOARDDIR)

BOARDSRC := \
        $(BOARDDIR)/board.o \
        $(BOARDDIR)/cfs-coffee-arch.o \
        $(BOARDDIR)/error.o \
        $(BOARDDIR)/frontend.o \
        $(BOARDDIR)/init.o \
        $(BOARDDIR)/acq.o \
        $(BOARDDIR)/usart_support.o \
        $(BOARDDIR)/nap/nap_dummy.o

BOARDASM := \
        $(BOARDDIR)/cpu_init.s \
        $(BOARDDIR)/mmu_table.s \

# Define linker script file here
LDSCRIPT= $(BOARDDIR)/ZYNQ7000.ld

RULESPATH = $(CHIBIOS)/os/common/ports/ARM/compilers/GCC

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-a9

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

#
# Compiler settings
##############################################################################


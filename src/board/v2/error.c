/*
 * Copyright (C) 2011-2015 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 * Contact: Henry Hallam <henry@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <hal.h>

#include <stdlib.h>
#include <string.h>

#include <libsbp/sbp.h>
#include <libswiftnav/logging.h>

#include "peripherals/leds.h"
#include "peripherals/usart.h"

#include "error.h"
#include "sbp.h"

#define TRAP_INT_DIV_ZERO
/* A lot of the SBP packet packing/unpacking fails involves unligned
   access at the moment, so the following is disabled for now*/
#undef TRAP_UNALIGNED_ACCESS


/** \addtogroup error
 * System low-level error handling and reporting
 * \{ */

/** \addtogroup io
 * \{ */

/** A simple DMA/interrupt-free UART write function, for use by screaming_death */
/* TODO: Move to peripherals/usart.c? */
static u32 fallback_write_ftdi(u8 *buff, u32 n, void *context)
{
  (void)context;
  for (u8 i=0; i<n; i++) {
    while (!(USART6->SR & USART_SR_TXE));
    USART6->DR = buff[i];
  }
  return n;
}
/** \} */

/** Error message.
 * Halts the program while continually sending a fixed error message in SBP
 * message format to the FTDI USART, in a way that should get the message
 * through to the Python console even if it's interrupting another transmission.
 *
 * \param msg A pointer to an array of chars containing the error message.
 */
void _screaming_death(const char *pos, const char *msg)
{
  __asm__("CPSID if;");           /* Disable all interrupts and faults */
  USART6->CR3 &= ~USART_CR3_DMAT;  /* Disable USART DMA */

  #define SPEAKING_MSG_N 222       /* Maximum length of error message */

  static char err_msg[SPEAKING_MSG_N] = "ERROR: ";

  strncat(err_msg, pos, SPEAKING_MSG_N - 8);
  strncat(err_msg, " : ", SPEAKING_MSG_N - strlen(err_msg) - 1);
  strncat(err_msg, msg, SPEAKING_MSG_N - strlen(err_msg) - 1);
  strncat(err_msg, "\n", SPEAKING_MSG_N - strlen(err_msg) - 1);
  u8 len = strlen(err_msg);

  static sbp_state_t sbp_state;
  sbp_state_init(&sbp_state);

  /* Continuously send error message */
  #define APPROX_ONE_SEC 33000000
  while (1) {
    led_toggle(LED_RED);
    for (u32 d = 0; d < APPROX_ONE_SEC; d++) {
      __asm__("nop");
    }
    /* TODO: Send to other UARTs? */
    sbp_send_message(&sbp_state, SBP_MSG_PRINT_DEP, 0, len, (u8*)err_msg, &fallback_write_ftdi);
  }
}


/* OS syscall implementations related to error conditions */

/** Custom assert() failure function. Calls screaming_death(). */
void __assert_func(const char *_file, int _line, const char *_func,
                   const char *_expr)
{
  char pos[255];
  char msg[255];
  sprintf(pos, "%s:%s():%d", _file, _func, _line);
  sprintf(msg, "assertion '%s' failed", _expr);
  _screaming_death(pos, msg);
}

/** Required by exit() which is (hopefully not) called from BLAS/LAPACK. */
void _fini(void)
{
  return;
}

/** _exit(2) syscall handler.  Called by (at least) abort() and exit().
 * Calls screaming_death() to repeatedly print an ERROR until WDT reset.
 */
void _exit(int status)
{
  (void)status;
  /* TODO: Perhaps print a backtrace; let's see if this ever actually
     occurs before implementing that. */
  screaming_death("abort() or exit() was called");
}

/** Enable and/or register handlers for system faults (hard fault, bus
 * fault, memory protection, usage (i.e. divide-by-zero) */
void fault_handling_setup(void) {
  /* Useful references:
     http://www.st.com/web/en/resource/technical/document/programming_manual/DM00046982.pdf
     http://chibios.sourceforge.net/docs/port_cmx_gcc_rm/nvic_8c.html */

  /* The following can be uncommented to separately handle hard
     faults, MPU faults, bus faults and usage faults.  Otherwise
     they'll all get promoted to hard faults.  If you uncomment this,
     implement MemManageVector, BusFaultVector and UsageFaultVector.

  nvicSetSystemHandlerPriority(HANDLER_MEM_MANAGE, 1);
  SCB_SHCSR |= (1 << 16);
  nvicSetSystemHandlerPriority(HANDLER_BUS_FAULT, 2);
  SCB_SHCSR |= (1 << 17);
  nvicSetSystemHandlerPriority(HANDLER_USAGE_FAULT, 3);
  SCB_SHCSR |= (1 << 18);
  */


#ifdef TRAP_UNALIGNED_ACCESS
  /* Enable trapping of unaligned accesses - the processor can perform
     most of them, but they're slow and may indicate a program
     error. */
  SCB->CCR |= (1 << 3);
#endif

#ifdef TRAP_INT_DIV_ZERO
  /* Enable trapping of integer division by zero - otherwise the
     operation will silently give a zero quotient. */
  SCB->CCR |= (1 << 4);  /* SCB_CCR_DIV_0_TRP */
}
#endif

/* The following CamelCase functions each override a 'weak' definition
   in ChibiOS-RT/os/ports/GCC/ARMCMx/STM32F4xx/vectors.c */
void NMIVector(void)
{
  /* We don't expect to ever end up here - On the STM32F4 the NMI can
     only be triggered by the RCC Clock Security System, which we
     don't enable. */
  screaming_death("NMI - RCC CSS?");
};


void HardFaultVector(void) __attribute__((noreturn));
void HardFaultVector(void)
{
  /* TODO: ChibiOS thread status dump
     TODO: Use MSG_PANIC to avoid sprintf
     TODO: Check that the master stack is in a sensible place, i.e. unlikely
           to conflict with the process stack
  */

  /* Retrieve the Process Stack Pointer, upon which is stacked important info
     (this handler is executing using the Master Stack Pointer instead).
     Get the Link Register as well, which is probably 0xFFFFFFED (see
     PM0214 section 2.3)
  */
  static uint32_t *psp, *lr;
  asm("mrs %0, psp" : "=r"(psp) : : );
  asm("mov %0, lr" : "=r"(lr) : : );
  
  static char msg[256];
  sprintf(msg, "HFSR=%08X CFSR=%08X MMFAR=%08X BFAR=%08X PSP=%08X LR=%08X "
          "r0=%08X r1=%08X r2=%08X r3=%08X r12=%08X lr=%08X pc=%08X psr=%08X",
          (unsigned int)SCB->HFSR, (unsigned int)SCB->CFSR,
          (unsigned int)SCB->MMFAR, (unsigned int)SCB->BFAR,
          (unsigned int)psp, (unsigned int)lr,
          (unsigned int)psp[0], (unsigned int)psp[1],
          (unsigned int)psp[2], (unsigned int)psp[3],
          (unsigned int)psp[4], (unsigned int)psp[5],
          (unsigned int)psp[6], (unsigned int)psp[7]);
  _screaming_death(__func__, msg);
};


/** \} */

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

#include "zynq7000.h"
#include <hal.h>

#include <stdlib.h>
#include <string.h>

#include <libsbp/sbp.h>
#include <libswiftnav/logging.h>

#include "peripherals/leds.h"
#include "peripherals/usart.h"

#include "error.h"
#include "sbp.h"

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
    while (UART1->SR & UART_SR_TXFULL_Msk);
    UART1->FIFO = buff[i];
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
}

__attribute__((interrupt("UNDEF")))
void Und_Handler(void)
{
  screaming_death("Undefined instruction!");
}

__attribute__((interrupt("ABORT")))
void Prefetch_Handler(void)
{
  screaming_death("Prefetch Abort!");
}

__attribute__((interrupt("ABORT")))
void Abort_Handler(void)
{
  screaming_death("Data Abort!");
}

__attribute__((interrupt("SWI")))
void Swi_Handler(void)
{
  screaming_death("Software Interrupt!");
}

__attribute__((interrupt("FIQ")))
void Fiq_Handler(void)
{
  screaming_death("Unused FIQ!");
}

/** \} */

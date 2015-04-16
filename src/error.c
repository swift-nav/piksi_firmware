/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/usart.h>
#include <stdlib.h>
#include <string.h>

#include <libsbp/sbp.h>

#include "board/leds.h"
#include "peripherals/usart.h"

#include "error.h"
#include "sbp.h"

/** \addtogroup io
 * \{ */

/** \addtogroup error
 * Last resort, low-level, blocking, continuous error messages.
 * \{ */

u32 fallback_write(u8 *buff, u32 n, void *context)
{
  (void)context;
  for (u8 i=0; i<n; i++) {
    while (!(USART6_SR & USART_SR_TXE));
    USART6_DR = buff[i];
  }
  return n;
}

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
  DMA2_S7CR = 0;                  /* Disable USART TX DMA */
  USART6_CR3 &= ~USART_CR3_DMAT;  /* Disable USART DMA */

  #define SPEAKING_MSG_N 128       /* Maximum length of error message */

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
    sbp_send_message(&sbp_state, SBP_MSG_PRINT, 0, len, (u8*)err_msg, &fallback_write);
    led_toggle(LED_RED);
    for (u32 d = 0; d < APPROX_ONE_SEC; d++) {
      __asm__("nop");
    }
  }
}

/** \} */

/** \} */

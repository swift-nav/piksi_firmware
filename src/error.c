/*
 * Copyright (C) 2011-2013 Swift Navigation Inc.
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

#include "board/leds.h"
#include "peripherals/usart.h"

#include "error.h"
#include "sbp.h"

/** \addtogroup io
 * \{ */

/** \addtogroup error
 * Last resort, low-level, blocking, continuous error messages.
 * \{ */

/** Error message.
 * Halts the program while continually sending a fixed error message in SBP
 * message format to the FTDI USART, in a way that should get the message
 * through to the Python console even if it's interrupting another transmission.
 *
 * \param msg A pointer to an array of chars containing the error message.
 */
void screaming_death(char *msg)
{
  __asm__("CPSID if;");           /* Disable all interrupts and faults */
  DMA2_S7CR = 0;                  /* Disable USART TX DMA */
  USART6_CR3 &= ~USART_CR3_DMAT;  /* Disable USART DMA */

  #define SPEAKING_MSG_N 64       /* Maximum length of error message */

  static char err_msg[SPEAKING_MSG_N+6] = {SBP_HEADER_1, SBP_HEADER_2,
                                           MSG_PRINT, SPEAKING_MSG_N,
                                           'E', 'R', 'R', 'O', 'R', ':', ' ',
                                           [11 ... SPEAKING_MSG_N+2] = '!',
                                           [SPEAKING_MSG_N+3] = '\n',
                                           /* CRC calculated below */
                                           [SPEAKING_MSG_N+4] = 0x00,
                                           [SPEAKING_MSG_N+5] = 0x00};

  /* Insert message */
  u8 i = 0;
  while (*msg && i < SPEAKING_MSG_N)  /* Don't want to use C library memcpy */
    err_msg[11 + (i++)] = *msg++;

  /* Insert CRC */
  u16 crc = crc16_ccitt((u8*)&err_msg[2], 2, 0);
  crc = crc16_ccitt((u8*)&err_msg[4], SPEAKING_MSG_N, crc);
  err_msg[SPEAKING_MSG_N + 4] = crc & 0xFF;
  err_msg[SPEAKING_MSG_N + 5] = (crc >> 8) & 0xFF;

  /* Continuously send error message */
  i = 0;
  while (1) {
    while (!(USART6_SR & USART_SR_TXE)) ;
    USART6_DR = err_msg[i];
    if (++i == (SPEAKING_MSG_N + 6)) {
      i = 0;
      led_toggle(LED_RED);
      for (u32 d = 0; d < 5000000; d++)
        __asm__("nop");
    }
  }
}

/** \} */

/** \} */


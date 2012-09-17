/*
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f4/dma.h>

#include "hw/leds.h"

#include "debug.h"
#include "error.h"

void screaming_death(void) {
  /* Disable ALL interrupts. */
  __asm__("CPSID if;");

  led_on(LED_RED);

  u8 error_string[] = "Error!\n";
  u8 i = 0;

  while(1) {
    while(error_string[i] != 0) {
      usart_send_blocking(USART1, error_string[i]);
      i++;
    }

    for (u32 i = 0; i < 600000; i++)
      __asm__("nop");
  }
};

/** Last resort, low-level error message function.  Halts the program while
 * continually sending a fixed string in debug message format, in a way that
 * should get the message through to the Python console even if it's
 * interrupting another transmission. */
void speaking_death(char *msg) {
  __asm__("CPSID if;");           /* Disable all interrupts and faults */
  DMA2_S7CR = 0;                  /* Disable USART TX DMA */
  USART1_CR3 &= ~USART_CR3_DMAT;  /* Disable USART DMA */

  #define ERR_MSG_N 64            /* Maximum length of error message */

  static char err_msg[ERR_MSG_N+4] = {DEBUG_MAGIC_1, DEBUG_MAGIC_2,
                                      MSG_PRINT, ERR_MSG_N,
                                      'E', 'R', 'R', 'O', 'R', ':', ' ',
                                      [11 ... ERR_MSG_N+3] = '!'};

  err_msg[ERR_MSG_N+3]='\n';

  u8 i=0;
  while (*msg && i < ERR_MSG_N)   /* Don't want to use C library memcpy */
    err_msg[11+(i++)] = *msg++;

  i=0;
  while (1) {
    while (!(USART1_SR & USART_SR_TXE));
    USART1_DR = err_msg[i];
    if (++i == (ERR_MSG_N + 4)) {
      i = 0;
      led_toggle(LED_RED);
      for (u32 d = 0; d < 5000000; d++)
        __asm__("nop");
    }
  }

}



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

#include "hw/leds.h"

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



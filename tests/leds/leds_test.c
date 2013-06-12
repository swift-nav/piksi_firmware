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

#include <stdio.h>

#include "main.h"
#include "board/leds.h"

int main(void)
{
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

	led_setup();
  led_on(LED_RED);
  led_on(LED_GREEN);

  while(1)
  {
    led_toggle(LED_RED);
    for (u32 i = 0; i < 600000; i++)
      __asm__("nop");
  }

  while (1);

	return 0;
}

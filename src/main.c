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

#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/gpio.h>
#include <stdio.h>

#include "debug.h"
#include "m25_flash.h"
#include "board/leds.h"

int main(void)
{
  u32 i, foo;
  u8 buff[100];

	led_setup();
  debug_setup();
  m25_setup();

  led_off(LED_GREEN);
  led_off(LED_RED);


	/* Blink the LEDs on the board. */
	while (1)
  {
		/* Using API function gpio_toggle(): */
		led_toggle(LED_GREEN);
    foo = m25_read_id();
		/*led_toggle(LED_RED);*/
    //DEBUG_MSG(MSG_U32, foo);
    m25_read(0x00, 0x06, buff);
    printf("Hello world %X %s\n", foo, buff);
    m25_read(0x10, 0x06, buff);
    printf("Hello world %X %s\n", foo, buff);
		for (i = 0; i < 600000; i++)	/* Wait a bit. */
			__asm__("nop");
	}

	return 0;
}

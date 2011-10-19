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

#include "debug.h"
#include "m25_flash.h"
#include "swift_nap.h"
#include "board/leds.h"
#include "board/exti.h"
#include "board/spi.h"

int main(void)
{
  u32 i, foo, bar = 0;
  //u8 buff[100];

	led_setup();
  debug_setup();
  //exti_setup();
  spi_setup();

  led_off(LED_GREEN);
  led_off(LED_RED);

  foo=bar;

  printf("Firmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");

	while (1)
  {
    foo++;

    swift_nap_write(1,0x69,foo);
    if ((bar = swift_nap_read(0,0x69)) != foo)
      printf("Expected %08X, got %08X\n", (unsigned int)foo, (unsigned int)bar);
/*
    foo = swift_nap_read(0x00, 0x22);
    if (foo == 0x01234567)
      ;//printf("");
    else
      printf("EIT 1\n");

    foo = swift_nap_read(0x01, 0x22);
    if (foo == 0xFFFFFFFF)
      printf("EIT 2\n");

    
    if (bar == 0) {
      bar = 1;
      swift_nap_write(0x01, 0x22, (0<<22));
    } else {
      bar = 0;
      swift_nap_write(0x01, 0x22, (1<<22));
    }

*/
    led_toggle(LED_RED);

    for (i = 0; i < 6; i++)
      __asm__("nop");
	}

	return 0;
}

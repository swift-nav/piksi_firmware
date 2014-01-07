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

#include "init.h"
#include "main.h"
#include "sbp.h"
#include "board/leds.h"
#include "peripherals/usart.h"

msg_callbacks_node_t foo_callback_node;
void foo_callback(u8 buff[])
{
  printf("Foo callback: %f\n", *((float*)buff));
}

msg_callbacks_node_t led_callback_node;
void led_callback(u8 buff[])
{
  printf("Green LED: ");
  if (buff[0] & 1) {
    led_on(LED_GREEN);
    printf("ON");
  } else {
    led_off(LED_GREEN);
    printf("OFF");
  }

  printf(", Red LED: ");
  if (buff[0] & 2) {
    led_on(LED_RED);
    printf("ON\n");
  } else {
    led_off(LED_RED);
    printf("OFF\n");
  }
}

int main(void)
{
  init(1);

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");
  printf("--- SWIFT BINARY PROTOCOL TEST ---\n");

  sbp_register_callback(0x22, &foo_callback, &foo_callback_node);
  sbp_register_callback(0x42, &led_callback, &led_callback_node);
  while(1)
  {
    sbp_process_messages();
    for (u32 i = 0; i < 600000; i++)
      __asm__("nop");
  }

  while (1);

	return 0;
}


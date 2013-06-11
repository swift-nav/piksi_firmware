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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <libopencm3/stm32/f4/rcc.h>

#include "main.h"
#include "init.h"
#include "error.h"
#include "sbp.h"
#include "board/leds.h"
#include "board/nap/nap_common.h"
#include "peripherals/usart.h"

#define MSG_ECHO 0xEC

void echo_callback(u8 buff[]){
  printf("%c\r",(char)buff[0]);
}

int main(void)
{
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

	led_setup();
  led_on(LED_GREEN);
  led_on(LED_RED);

  nap_conf_b_setup();
  nap_conf_b_clear();

  sbp_setup(0);

  static msg_callbacks_node_t echo_node;
  sbp_register_callback(MSG_ECHO, &echo_callback, &echo_node);

  while(1) {
    DO_EVERY(3000,
      led_toggle(LED_RED);
      led_toggle(LED_GREEN);
    );
    sbp_process_messages();
  }

  while (1);

	return 0;
}


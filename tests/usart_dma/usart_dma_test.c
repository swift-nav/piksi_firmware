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

#include "init.h"
#include "error.h"
#include "hw/leds.h"
#include "debug.h"
#include "hw/usart.h"
#include "swift_nap_io.h"

#define MSG_ECHO 0xEC

//void echo_callback(u8 buff[]){
//  printf("%c\r",(char)buff[0]);
//}

int main(void)
{
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

	led_setup();
  led_on(LED_GREEN);
  led_on(LED_RED);

  swift_nap_conf_b_setup();
  swift_nap_conf_b_clear();

  debug_setup(0);

//  static msg_callbacks_node_t echo_node;
//  debug_register_callback(MSG_ECHO, &echo_callback, &echo_node);

  #define CHECK_N 500
  u8 received_chars[CHECK_N];
  u8 expected_chars[CHECK_N];
  for (u16 i=0;i<CHECK_N;i++){
    expected_chars[i] = i;
  }
  u64 n_received = 0;
  u32 n_read;

  while(1) {
//    if (usart_n_read_dma(&ftdi_rx_state)) {
    n_read = usart_read_dma(&ftdi_rx_state,received_chars,CHECK_N);
    while (!n_read){
      n_read = usart_read_dma(&ftdi_rx_state,received_chars,CHECK_N);
    }
    for (u16 i=0;i<CHECK_N;i++) {
      if (received_chars[i] != expected_chars[i])
        speaking_death("NOT EQUAL =( ");
    }
    for (u16 i=0;i<CHECK_N;i++){
      expected_chars[i] = expected_chars[CHECK_N-1]+1+i;
    }
    n_received += CHECK_N;
//    }
//    DO_EVERY(30,
      led_toggle(LED_RED);
      led_toggle(LED_GREEN);
//    );
//    debug_process_messages();
  }

  while (1);

	return 0;
}


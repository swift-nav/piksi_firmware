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

#include <stdio.h>
#include <libswiftnav/sbp.h>

#include "init.h"
#include "main.h"
#include "sbp.h"
#include "board/leds.h"
#include "peripherals/usart.h"

sbp_msg_callbacks_node_t foo_callback_node;
void foo_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void) context;
  printf("Foo callback: %f\n", *((float*)msg));
}

sbp_msg_callbacks_node_t led_callback_node;
void led_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void) context;

  printf("Green LED: ");
  if (msg[0] & 1) {
    led_on(LED_GREEN);
    printf("ON");
  } else {
    led_off(LED_GREEN);
    printf("OFF");
  }

  printf(", Red LED: ");
  if (msg[0] & 2) {
    led_on(LED_RED);
    printf("ON\n");
  } else {
    led_off(LED_RED);
    printf("OFF\n");
  }
}

int main(void)
{
  init();

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");
  printf("--- SWIFT BINARY PROTOCOL TEST ---\n");

  sbp_register_cbk(0x22, &foo_callback, &foo_callback_node);
  sbp_register_cbk(0x42, &led_callback, &led_callback_node);
  while(1)
  {
    sbp_process_messages();
    for (u32 i = 0; i < 600000; i++)
      __asm__("nop");
  }

  while (1);

	return 0;
}


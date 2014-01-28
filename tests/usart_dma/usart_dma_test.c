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

#include "main.h"
#include "sbp.h"
#include "board/leds.h"
#include "board/nap/nap_common.h"

int main(void)
{
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

	led_setup();
  led_on(LED_GREEN);
  led_on(LED_RED);

  /* NAP is not required for this test. */
  nap_conf_b_setup();
  nap_conf_b_clear();

  sbp_setup(0, 0);

  while (1) {
    led_toggle(LED_RED);
    led_toggle(LED_GREEN);
    for (int i = 0; i < 10000; i++) /* Wait a bit. */
      __asm__("NOP");
    printf("ABCDEFGHIJKLMNOPQRSTUVWXYZ\n\r");
  }

	return 0;
}


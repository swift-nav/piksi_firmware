/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdio.h>
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/flash.h>
#include <libopencm3/stm32/f2/gpio.h>

#include "init.h"
#include "main.h"
#include "sbp.h"
#include "acq.h"
#include "board/leds.h"
#include "board/m25_flash.h"
#include "board/nap/nap_common.h"
#include "peripherals/spi.h"

int main(void)
{
  init();

  led_on(LED_GREEN);
  led_on(LED_RED);

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");
  printf("--- DNA TEST ---\n");

  while (1) {
    for (u32 i = 0; i < 10000; i++)
      __asm__("nop");
    DO_EVERY(50,
      led_toggle(LED_GREEN);
      led_toggle(LED_RED);
    );
    sbp_process_messages();
  }

  return 0;
}

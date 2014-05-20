/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdio.h>

#include "init.h"
#include "main.h"
#include "sbp.h"
#include "acq.h"
#include "board/leds.h"
#include "board/m25_flash.h"
#include "board/nap/nap_common.h"

int main(void)
{

  init();

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n\r");
  printf("--- TIMING STROBE TEST ---\n\r");

  u32 tcs;
  u32 tcls;
  u32 strobe_offset = 2000;

  tcs = nap_timing_count();
  acq_schedule_load(tcs + strobe_offset);
  tcls = nap_timing_count_latched();

  printf("nap_timing_count[%d]          = %u\n", 0, (unsigned int)tcs);
  printf("nap_timing_count_latched[%d]  = %u\n", 0, (unsigned int)tcls);
  printf("difference = %u\n", (unsigned int)(tcls-tcs));

  led_off(LED_GREEN);
  led_on(LED_RED);

  while(1);

	return 0;
}


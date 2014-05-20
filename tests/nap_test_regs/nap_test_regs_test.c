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

#include "init.h"
#include "main.h"
#include "sbp.h"
#include "acq.h"
#include "board/leds.h"
#include "board/nap/nap_common.h"

int main(void)
{
  init();

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n\r");
  printf("--- NAP TEST REGS TEST ---\n\r");

  u8 rdwr_in[4];
  u8 rdwr_out[4] = {0, 0, 0, 0};

  u32 count = 1;
  u32 rdwr_bad = 0;
  u32 beef_bad = 0;

  /* First read should give "DECAFBAD", */
  nap_xfer_blocking(254, 4, rdwr_in, rdwr_out);
  if (rdwr_in[0] != 0xDE ||
      rdwr_in[1] != 0xCA ||
      rdwr_in[2] != 0xFB ||
      rdwr_in[3] != 0xAD) {
    printf("First read of rd/wr test register reads %02X%02X%02X%02X, "
           "should be DECAFBAD\n",
           rdwr_in[0], rdwr_in[1],
           rdwr_in[2], rdwr_in[3]);
    rdwr_bad++;
  }

  u8 i = 1;
  while (1) {
    rdwr_out[0] = i;
    rdwr_out[1] = i;
    rdwr_out[2] = i;
    rdwr_out[3] = i;
    nap_xfer_blocking(254, 4, rdwr_in, rdwr_out);
    if (rdwr_in[0] != (u8)(i-1) ||
        rdwr_in[1] != (u8)(i-1) ||
        rdwr_in[2] != (u8)(i-1) ||
        rdwr_in[3] != (u8)(i-1)) {
      printf("\nrd/wr register reads %02X%02X%02X%02X, "
             " but i = 0x%02X\n",
             rdwr_in[0], rdwr_in[1],
             rdwr_in[2], rdwr_in[3], (u8)(i-1));
      rdwr_bad++;
    }
    i++;

    nap_xfer_blocking(255, 4, rdwr_in, NULL);
    if (rdwr_in[0] != 0xDE ||
        rdwr_in[1] != 0xAD ||
        rdwr_in[2] != 0xBE ||
        rdwr_in[3] != 0xEF) {
      printf("\nDeceased cow register reads %02X%02X%02X%02X, "
             "should be DEADBEEF\n",
             rdwr_in[0], rdwr_in[1],
             rdwr_in[2], rdwr_in[3]);
      beef_bad++;
    }

    led_toggle(LED_GREEN);
    led_toggle(LED_RED);

    count++;

    if (count % 5000 == 0)
      printf("%d/%d errors out of %d\n",
          (unsigned int)rdwr_bad, (unsigned int)beef_bad, (unsigned int)count);

    /*for (u32 i = 0; i < 60000; i++)*/
      /*__asm__("nop");*/

  }

  while (1);

	return 0;
}


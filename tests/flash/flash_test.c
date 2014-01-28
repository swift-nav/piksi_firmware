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
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/flash.h>
#include <libopencm3/stm32/f4/gpio.h>

#include "main.h"
#include "sbp.h"
#include "acq.h"
#include "board/leds.h"
#include "board/nap/nap_common.h"
#include "peripherals/spi.h"
#include "flash_callbacks.h"

int main(void)
{

  led_setup();
  led_on(LED_GREEN);
  led_on(LED_RED);

  /* Setup and hold the FPGA PROGRAM_B line low so that the FPGA does not
   * contest the flash SPI bus */
  nap_conf_b_setup();
  nap_conf_b_clear();

  spi_setup();
  sbp_setup(0, 0);
  flash_callbacks_register();

  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");
  printf("--- M25 FLASH TEST ---\n");

  while (1) {
    DO_EVERY(300,
      led_toggle(LED_GREEN);
      led_toggle(LED_RED);
    );
    sbp_process_messages();
  }

  return 0;
}

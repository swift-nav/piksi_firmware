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
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/flash.h>
#include <libopencm3/stm32/f4/usart.h>
#include <libopencm3/stm32/f2/gpio.h>

#include "main.h"
#include "debug.h"
#include "swift_nap_io.h"
#include "acq.h"
#include "hw/leds.h"

const clock_scale_t hse_16_368MHz_in_65_472MHz_out_3v3 =
{ /* 65.472 MHz */
  .pllm = 16,
  .plln = 256,
  .pllp = 4,
  .pllq = 6,
  .hpre = RCC_CFGR_HPRE_DIV_NONE,
  .ppre1 = RCC_CFGR_PPRE_DIV_4,
  .ppre2 = RCC_CFGR_PPRE_DIV_4,
  .flash_config = FLASH_ICE | FLASH_DCE | FLASH_LATENCY_2WS,
  .apb1_frequency = 16368000,
  .apb2_frequency = 16368000,
};

int main(void)
{
	led_setup();

  led_off(LED_GREEN);
  led_on(LED_RED);

  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

  led_off(LED_GREEN);
  led_off(LED_RED);

  swift_nap_setup();
  swift_nap_reset();

  led_on(LED_GREEN);
  led_off(LED_RED);

  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_65_472MHz_out_3v3);

  debug_setup();

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n\r");
  printf("--- NAP TEST REGS TEST ---\n\r");

  u8 rdwr_in[4];
  u8 rdwr_out[4] = {0, 0, 0, 0};

  u32 count = 1;
  u32 rdwr_bad = 0;
  u32 beef_bad = 0;

  /* First read should give "DECAFBAD", */
  swift_nap_xfer_blocking(254, 4, rdwr_in, rdwr_out);
  if (rdwr_in[0] != 0xDD ||
      rdwr_in[1] != 0xCA ||
      rdwr_in[2] != 0xFB ||
      rdwr_in[3] != 0xAD) {
    printf("First read of rd/wr test register reads %02X%02X%02X%02X, "
           "should be DECAFBAD\n",
           (u16)rdwr_in[0], (u16)rdwr_in[1],
           (u16)rdwr_in[2], (u16)rdwr_in[3]);
    rdwr_bad++;
  }

  u8 i = 1;
  while (1) {
    rdwr_out[0] = i;
    rdwr_out[1] = i;
    rdwr_out[2] = i;
    rdwr_out[3] = i;
    swift_nap_xfer_blocking(254, 4, rdwr_in, rdwr_out);
    if (rdwr_in[0] != (u8)(i-1) ||
        rdwr_in[1] != (u8)(i-1) ||
        rdwr_in[2] != (u8)(i-1) ||
        rdwr_in[3] != (u8)(i-1)) {
      printf("\nrd/wr register reads %02X%02X%02X%02X, "
             " but i = 0x%02X\n",
             (u16)rdwr_in[0], (u16)rdwr_in[1],
             (u16)rdwr_in[2], (u16)rdwr_in[3], (u8)(i-1));
      rdwr_bad++;
    }
    i++;

    swift_nap_xfer_blocking(255, 4, rdwr_in, NULL);
    if (rdwr_in[0] != 0xDE ||
        rdwr_in[1] != 0xAD ||
        rdwr_in[2] != 0xBE ||
        rdwr_in[3] != 0xEF) {
      printf("\nDeceased cow register reads %02X%02X%02X%02X, "
             "should be DEADBEEF\n",
             (u16)rdwr_in[0], (u16)rdwr_in[1],
             (u16)rdwr_in[2], (u16)rdwr_in[3]);
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


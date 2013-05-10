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
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/flash.h>
#include <libopencm3/stm32/f4/gpio.h>

#include "main.h"
#include "debug.h"
#include "swift_nap_io.h"
#include "acq.h"
#include "hw/leds.h"
#include "hw/m25_flash.h"

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
  debug_setup(1);

  swift_nap_setup();
  swift_nap_reset();

  led_on(LED_GREEN);
  led_on(LED_RED);
  
  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_65_472MHz_out_3v3);

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n\r");
  printf("--- TIMING STROBE TEST ---\n\r");

  u32 tcs;
  u32 tcls;
  u32 strobe_offset = 2000;

  tcs = timing_count();
  acq_schedule_load(tcs + strobe_offset);
  tcls = timing_count_latched();

  printf("timing_count[%d]          = %u\n", 0, (unsigned int)tcs);
  printf("timing_count_latched[%d]  = %u\n", 0, (unsigned int)tcls);
  printf("difference = %u\n", (unsigned int)(tcls-tcs));
  
  led_off(LED_GREEN);
  led_on(LED_RED);

  while(1);

	return 0;
}


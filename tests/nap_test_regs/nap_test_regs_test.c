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
  u8 rdwr_out[4];

  for (u8 i = 0; i < 6; i++){
    rdwr_out[0] = i;
    rdwr_out[1] = i;
    rdwr_out[2] = i;
    rdwr_out[3] = i;
    swift_nap_xfer_blocking(254,4,rdwr_in,rdwr_out);
    printf("rdwr_in reads %x%x%x%x\n",(u16)rdwr_in[0],(u16)rdwr_in[1],(u16)rdwr_in[2],(u16)rdwr_in[3]);
  }

  u8 beef[4] = {0,1,2,3};
  swift_nap_xfer_blocking(255,4,beef,NULL);
  printf("register 255 reads %x%x%x%x\n",(u16)beef[0],(u16)beef[1],(u16)beef[2],(u16)beef[3]);

  led_on(LED_GREEN);
  led_on(LED_RED);

  while (1);

	return 0;
}


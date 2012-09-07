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
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

	led_setup();

  led_on(LED_GREEN);
  led_on(LED_RED);

  /* Setup MAX2769 PGM (PB8) - low */
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPBEN;
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO8);
  gpio_clear(GPIOB, GPIO8);

  /* Setup MAX2769 NSHDN (PB9) - high */
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO9);
  gpio_set(GPIOB, GPIO9);

  /* Setup MAX2769 NIDLE (PB10) - high */
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO10);
  gpio_set(GPIOB, GPIO10);

  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_65_472MHz_out_3v3);

  debug_setup();

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n\r");
  printf("--- ACQ TEST ---\n\r");

  swift_nap_setup();
  swift_nap_reset();

  led_toggle(LED_GREEN);
  led_toggle(LED_RED);

  float code_phase;
  float carrier_freq;
  float snr;

  acq_schedule_load(timing_count() + 1000);
  while(!(acq_get_load_done()));
  led_off(LED_GREEN);
  led_off(LED_RED);

  for (u8 prn=0; prn<32; prn++) {
    acq_write_code_blocking(prn);
    acq_start(prn, 0, 1023, -7000, 7000, 300);
    while(!(acq_get_done()));
    acq_get_results(&code_phase, &carrier_freq, &snr);

    printf("PRN %2u - Code phase: %7.2f, Carrier freq: % 7.1f, SNR: %5.2f", prn+1, code_phase, carrier_freq, snr);
    if (snr > 50.0)
      printf("   :D\n");
    else
      printf("\n");
    led_toggle(LED_GREEN);
    led_toggle(LED_RED);
  }

  printf("DONE!\n");
  led_on(LED_GREEN);
  led_off(LED_RED);
  while (1);

	return 0;
}


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
#include <math.h>
#include <string.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/flash.h>
#include <libopencm3/stm32/f4/gpio.h>

#include "main.h"
#include "debug.h"
#include "swift_nap_io.h"
#include "track.h"
#include "acq.h"
#include "cw.h"
#include "manage.h"
#include "hw/leds.h"
#include "hw/spi.h"
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

const clock_scale_t hse_16_368MHz_in_130_944MHz_out_3v3 =
{ /* 130.944 MHz (Overclocked!!) */
  .pllm = 16,
  .plln = 256,
  .pllp = 2,
  .pllq = 6,
  .hpre = RCC_CFGR_HPRE_DIV_NONE,
  .ppre1 = RCC_CFGR_PPRE_DIV_8,
  .ppre2 = RCC_CFGR_PPRE_DIV_4,
  .flash_config = FLASH_ICE | FLASH_DCE | FLASH_LATENCY_3WS,
  .apb1_frequency = 16368000,
  .apb2_frequency = 2*16368000,
};

const clock_scale_t hse_16_368MHz_in_120_203MHz_out_3v3 =
{ /* 120.203 MHz (USB OK but APB freq */
  .pllm = 16,
  .plln = 235,
  .pllp = 2,
  .pllq = 5,
  .hpre = RCC_CFGR_HPRE_DIV_2,
  .ppre1 = RCC_CFGR_PPRE_DIV_2,
  .ppre2 = RCC_CFGR_PPRE_DIV_2,
  .flash_config = FLASH_ICE | FLASH_DCE | FLASH_LATENCY_3WS,
  .apb1_frequency = 30050625,
  .apb2_frequency = 30050625,
};

int main(void)
{
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

	led_setup();

  // Debug pins (CC1111 TX/RX)
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10|GPIO11);
  gpio_clear(GPIOC, GPIO10|GPIO11);

  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_130_944MHz_out_3v3);

  debug_setup();

  printf("\n\n# Firmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");

  swift_nap_setup();
  swift_nap_reset();

  m25_setup();
  xfer_dna_hash();

  led_toggle(LED_GREEN);
  led_toggle(LED_RED);

	u64 cw_power;
	float cw_freq;

	while(1) {

		printf("#PLOT_DATA_START\n");

		// Load CW ram
		cw_schedule_load(timing_count() + 1000);
		while (!(cw_get_load_done()));
		printf("# Finished loading cw ram\n");

		// Do CW detection
//		cw_start(-4e6,4e6,8e6/(SPECTRUM_LEN-1));
//		cw_start(0.5e6,0.7e6,0.2e6/(SPECTRUM_LEN-1));
    float cf = (1.575542*1e9-1.575420*1e9);
    float span = 200e3;
		cw_start(cf-span/2,cf+span/2,span/(SPECTRUM_LEN-1));
		while (!(cw_get_running_done()));
		printf("# Finished doing cw detection\n");

		for (u16 si=0;si<SPECTRUM_LEN;si++) {

			for (u32 dly = 0; dly < 50000; dly++)
				__asm__("nop");

			cw_get_spectrum_point(&cw_freq,&cw_power,si);

      if (~((cw_power == 0) && (cw_freq == 0))) {
//			printf("%+7.2f %lu # %d\n",cw_freq,(long unsigned int)cw_power,(unsigned int)si);
			printf("%+7.1f %lu\n",cw_freq,(long unsigned int)cw_power);
//			printf("%+4.2f %lu\n",cw_freq,(long unsigned int)cw_power);
      }

			u32 err = swift_nap_read_error_blocking();
			if (err) {
				printf("Error: 0x%08X\n", (unsigned int)err);
				while(1);
			}
		}

		printf("#PLOT_DATA_END\n");

	}

  while (1);

	return 0;
}


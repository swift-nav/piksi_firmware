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
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/dma.h>
#include <libopencm3/stm32/f2/flash.h>
#include <libopencm3/stm32/f2/gpio.h>

#include "main.h"
#include "debug.h"
#include "swift_nap_io.h"
#include "track.h"
#include "acq.h"
#include "hw/leds.h"
#include "hw/spi.h"

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
  .hpre = RCC_CFGR_HPRE_DIV_2,
  .ppre1 = RCC_CFGR_PPRE_DIV_4,
  .ppre2 = RCC_CFGR_PPRE_DIV_4,
  .flash_config = FLASH_ICE | FLASH_DCE | FLASH_LATENCY_3WS,
  .apb1_frequency = 16368000,
  .apb2_frequency = 16368000,
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

  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_65_472MHz_out_3v3);

  debug_setup();

  printf("\n\n# Firmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");

  swift_nap_setup();
  swift_nap_reset();
 
  led_toggle(LED_RED);
  
  u8 prn = 31-1;

  /* Initial coarse acq. */
  float acq_code_phase;
  float acq_carrier_freq;
  float acq_snr;
  u32 acq_count = acq_full_two_stage(prn, &acq_code_phase, &acq_carrier_freq, &acq_snr);

  printf("# PRN %u: %f, %f, %f\n", prn+1, acq_code_phase, acq_carrier_freq, acq_snr);

  if (acq_snr < ACQ_THRESHOLD) {
    printf("No findy satellite :(\n");
    while(1);
  }

  /* Transition to tracking. */
  u32 track_count = timing_count() + 20000;
  float track_cp = propagate_code_phase(acq_code_phase, acq_carrier_freq, track_count - acq_count);

  tracking_channel_init(1, prn, track_cp, acq_carrier_freq, track_count);

  prn = 1-1;

  acq_count = acq_full_two_stage(prn, &acq_code_phase, &acq_carrier_freq, &acq_snr);

  printf("# PRN %u: %f, %f, %f\n", prn+1, acq_code_phase, acq_carrier_freq, acq_snr);

  u32 track_count2 = timing_count() + 20000;
  float track_cp2 = propagate_code_phase(acq_code_phase, acq_carrier_freq, track_count2 - acq_count);

  tracking_channel_init(2, prn, track_cp2, acq_carrier_freq, track_count2);

  while(1)
  {
    for (u32 i = 0; i < 1000000; i++)
      __asm__("nop");

    for (u8 i=0; i<TRACK_N_CHANNELS; i++) {
      switch (tracking_channel[i].state) {
        default:
        case TRACKING_DISABLED:
          printf("X\t");
          break;
        case TRACKING_FIRST_LOOP:
          printf("F\t");
          break;
        case TRACKING_RUNNING:
          printf("%.2f\t", tracking_channel_snr(i));
          break;
      }
    }
    printf("\n");
    u32 err = swift_nap_read_error_blocking();
    if (err)
      printf("Error: 0x%08X\n", (unsigned int)err);

  }

  while (1);
  
	return 0;
}


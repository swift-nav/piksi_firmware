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
  
  u8 prn = 32-1;

  /* Initial coarse acq. */
  float coarse_acq_code_phase;
  float coarse_acq_carrier_freq;
  float coarse_snr;
  u32 coarse_acq_cnt = timing_count() + 1000;
  acq_schedule_load(coarse_acq_cnt);
  acq_wait_load_done();

  do_acq(prn, 0, 1023, -7000, 7000, 300, &coarse_acq_code_phase, &coarse_acq_carrier_freq, &coarse_snr);
  printf("#Coarse - PRN %u: %f, %f, %f\n", prn+1, coarse_acq_code_phase, coarse_acq_carrier_freq, coarse_snr);

  if (coarse_snr < 8.0) {
    printf("No findy satellite :(\n");
    while(1);
  }

  /* Fine acq. */
  float fine_acq_code_phase;
  float fine_acq_carrier_freq;
  float fine_snr;
  u32 fine_acq_cnt = timing_count() + 2000;
  acq_schedule_load(fine_acq_cnt);
  acq_wait_load_done();

  float fine_cp = propagate_code_phase(coarse_acq_code_phase, coarse_acq_carrier_freq, fine_acq_cnt - coarse_acq_cnt);

  do_acq(prn, fine_cp-20, fine_cp+20, coarse_acq_carrier_freq-300, coarse_acq_carrier_freq+300, 30, &fine_acq_code_phase, &fine_acq_carrier_freq, &fine_snr);

  printf("#Fine - PRN %u: (%f) %f, %f, %f\n", prn+1, fine_cp, fine_acq_code_phase, fine_acq_carrier_freq, fine_snr);


  /* Transition to tracking. */
  u32 track_cnt = timing_count() + 20000;
  float track_cp = propagate_code_phase(fine_acq_code_phase, fine_acq_carrier_freq, track_cnt - fine_acq_cnt);

  tracking_channel_init(1, prn, track_cp, fine_acq_carrier_freq, track_cnt);
  tracking_channel_init(2, prn, track_cp, fine_acq_carrier_freq, track_cnt);

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
    __asm__("CPSID i;");
    printf("Error: 0x%08X\n", (unsigned int)swift_nap_read_error_blocking());
    __asm__("CPSIE i;");

  }

  while (1);
  
	return 0;
}


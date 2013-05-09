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
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/flash.h>
#include <libopencm3/stm32/f4/gpio.h>

#include "main.h"
#include "debug.h"
#include "swift_nap_io.h"
#include "track.h"
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
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

	led_setup();

  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_65_472MHz_out_3v3);

  debug_setup();

  #ifndef PRN
    #error Please define the PRN to be used, e.g. make PRN=22
  #endif

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");
  printf("--- CODE PHASE PROPAGATION TEST ---\n");

  swift_nap_setup();
  swift_nap_reset();

  m25_setup();

  while(1) {
    printf("\nPRN: %u\n", PRN);

    /* Initial coarse acq. */
    float coarse_acq_code_phase;
    float coarse_acq_carrier_freq;
    float coarse_snr;
    acq_set_load_enable_blocking();
    u32 coarse_acq_cnt = timing_count() + 1000;
    timing_strobe(coarse_acq_cnt);
    wait_for_exti();
    acq_clear_load_enable_blocking();

    do_acq(PRN-1, 0, 1023, -7000, 7000, 300, &coarse_acq_code_phase, &coarse_acq_carrier_freq, &coarse_snr);
    printf("Coarse:\n  Code phase: %7.2f, Carrier freq % 7.1f, SNR %5.2f\n", coarse_acq_code_phase, coarse_acq_carrier_freq, coarse_snr);

    if (coarse_snr < 8.0) {
      printf("  Didn't acquire satellite :(\n");
      led_on(LED_RED);
      continue;
    }

    /* Fine acq. */
    float fine_acq_code_phase;
    float fine_acq_carrier_freq;
    float fine_snr;
    acq_set_load_enable_blocking();
    u32 fine_acq_cnt = timing_count() + 2000;
    timing_strobe(fine_acq_cnt);
    wait_for_exti();
    acq_clear_load_enable_blocking();

    float fine_cp = propagate_code_phase(coarse_acq_code_phase, coarse_acq_carrier_freq, fine_acq_cnt - coarse_acq_cnt);

    do_acq(PRN-1, fine_cp-20, fine_cp+20, coarse_acq_carrier_freq-300, coarse_acq_carrier_freq+300, 100, &fine_acq_code_phase, &fine_acq_carrier_freq, &fine_snr);

    float dt = (float)(fine_acq_cnt - coarse_acq_cnt) * 1000.0 / SAMPLE_FREQ;
    printf("Fine:\n  Propagated for %.1f ms, propagated code phase %.4f\n", dt, fine_cp);
    printf("  Code phase: %7.4f, Carrier freq % 7.1f, SNR %5.2f\n", fine_acq_code_phase, fine_acq_carrier_freq, fine_snr);
    printf("  Code phase error: %f\n", fine_acq_code_phase - fine_cp);

    /* Second fine acq. */
    float fine2_acq_code_phase;
    float fine2_acq_carrier_freq;
    float fine2_snr;
    acq_set_load_enable_blocking();
    u32 fine2_acq_cnt = timing_count() + 2000;
    timing_strobe(fine2_acq_cnt);
    wait_for_exti();
    acq_clear_load_enable_blocking();

    float fine2_cp = propagate_code_phase(fine_acq_code_phase, fine_acq_carrier_freq, fine2_acq_cnt - fine_acq_cnt);

    do_acq(PRN-1, fine2_cp-20, fine2_cp+20, fine_acq_carrier_freq-300, fine_acq_carrier_freq+300, 100, &fine2_acq_code_phase, &fine2_acq_carrier_freq, &fine2_snr);

    dt = (float)(fine2_acq_cnt - fine_acq_cnt) * 1000.0 / SAMPLE_FREQ;
    printf("Second fine:\n  Propagated for %.1f ms, propagated code phase %.4f\n", dt, fine2_cp);
    printf("  Code phase: %7.4f, Carrier freq % 7.1f, SNR %5.2f\n", fine2_acq_code_phase, fine2_acq_carrier_freq, fine2_snr);
    printf("  Code phase error: %f\n", fine2_acq_code_phase - fine2_cp);
  }

	return 0;
}


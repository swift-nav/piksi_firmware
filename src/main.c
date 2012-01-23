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
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/flash.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f2/gpio.h>

#include "main.h"
#include "debug.h"
#include "max2769.h"
#include "m25_flash.h"
#include "swift_nap.h"
#include "board/leds.h"
#include "board/exti.h"
#include "board/spi.h"

#define DLL_IGAIN 1.431702e-2
#define DLL_PGAIN 5.297297
#define PLL_IGAIN 1.779535e+1
#define PLL_PGAIN 3.025210e+2


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
  led_off(LED_GREEN);
  led_off(LED_RED);

  // Debug pins (CC1111 TX/RX)
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10|GPIO11);
  gpio_clear(GPIOC, GPIO10|GPIO11);

  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_65_472MHz_out_3v3);
  /*rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_130_944MHz_out_3v3);*/

  debug_setup();

  printf("\n\n# Firmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");

  propagate_code_phase(664, -550, 16368000);
  while(1);

  spi_setup();
  max2769_setup();
  swift_nap_setup();
  swift_nap_reset();
  exti_setup();
  
  led_toggle(LED_RED);

  acq_set_load_enable();
  u32 acq_cnt = timing_count() + 1000;
  timing_strobe(acq_cnt);
  wait_for_exti();
  acq_clear_load_enable();

  float acq_code_phase;
  float acq_carrier_freq;
  float snr;

  for (u8 prn=18; prn<21; prn++) {
    do_acq(prn, 0, 1023, -10000, 10000, &acq_code_phase, &acq_carrier_freq, &snr);
    printf("#PRN %u: %f, %f, %f\n", prn+1, acq_code_phase, acq_carrier_freq, snr);
  }
  while(1);

  u32 code_phase_rate = (1.0 + acq_carrier_freq/L1_HZ) * TRACK_NOMINAL_CODE_PHASE_RATE;
  printf("#CPR: %u\n", (unsigned int)code_phase_rate);

  /*while(1);*/

  u32 track_cnt = timing_count() + 2000;

  float track_cp = propagate_code_phase(acq_code_phase, acq_carrier_freq, track_cnt - acq_cnt);

  tracking_channel_init(21, 0, track_cp, acq_carrier_freq, track_cnt);

  corr_t cs[100][3];

  // Do one open loop update as first set of correlations
  // are junk.
  wait_for_exti();
  track_read_corr(0, cs[0]);
  track_write_update(0, acq_carrier_freq*TRACK_CARRIER_FREQ_UNITS_PER_HZ, code_phase_rate);

  double dll_disc = 0;
  double pll_disc = 0;
  double dll_disc_old;
  double pll_disc_old;

  double dll_freq = code_phase_rate / TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;
  double pll_freq = acq_carrier_freq;

  u32 dll_freq_fp;
  s16 pll_freq_fp;

  for (u8 n=0; n<100; n++) {
    wait_for_exti();
    track_read_corr(0, cs[n]);

    dll_disc_old = dll_disc;
    pll_disc_old = pll_disc;

    // TODO: check for divide by zero
    pll_disc = atan((double)cs[n][1].Q/(double)cs[n][1].I)/(2*3.14159);

    pll_freq = pll_freq + PLL_PGAIN*(pll_disc-pll_disc_old) \
               + PLL_IGAIN*pll_disc;

    dll_disc = (sqrt((double)cs[n][0].I*(double)cs[n][0].I + (double)cs[n][0].Q*(double)cs[n][0].Q) - sqrt((double)cs[n][2].I*(double)cs[n][2].I + (double)cs[n][2].Q*(double)cs[n][2].Q)) \
               / (sqrt((double)cs[n][0].I*(double)cs[n][0].I + (double)cs[n][0].Q*(double)cs[n][0].Q) + sqrt((double)cs[n][2].I*(double)cs[n][2].I + (double)cs[n][2].Q*(double)cs[n][2].Q));

    dll_freq = dll_freq + DLL_PGAIN*(dll_disc-dll_disc_old) \
               + DLL_IGAIN*dll_disc;

    pll_freq_fp = (s16)(round(pll_freq)*pow(2,24)/SAMPLE_FREQ);
    dll_freq_fp = (u32)(round(dll_freq)*pow(2,32)/SAMPLE_FREQ);

    /*printf("%u, %d\n", (unsigned int)dll_freq_fp, (int)pll_freq_fp);*/

    track_write_update(0, pll_freq_fp, dll_freq_fp);
  }

  printf("# prop phase: %f\n", track_cp);
  printf("foo = [\n");
  for (u8 n=0; n<100; n++)
    printf("(%6d,%6d),\n", (int)cs[n][1].I, (int)cs[n][1].Q);
  printf("]\n");

  track_read_corr(0, cs[0]);

  while (1);
  
	return 0;
}


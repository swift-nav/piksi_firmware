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
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

  led_setup();

  led_on(LED_GREEN);
  led_on(LED_RED);

  debug_setup();

  swift_nap_setup();
  swift_nap_reset();

  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_65_472MHz_out_3v3);

  m25_setup();

//  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n\r");
//  printf("--- ACQ TEST ---\n\r");

  printf("ACQ_N_TAPS = %d\n", (unsigned int)ACQ_N_TAPS); for (u64 i=0; i<1000; i++) __asm__("nop");
  printf("ACQ_CODE_PHASE_WIDTH = %d\n",(unsigned int)ACQ_CODE_PHASE_WIDTH); for (u64 i=0; i<1000; i++) __asm__("nop");
  printf("ACQ_CARRIER_FREQ_WIDTH = %d\n",(unsigned int)ACQ_CARRIER_FREQ_WIDTH); for (u64 i=0; i<1000; i++) __asm__("nop");
  printf("TRACK_INIT_CODE_PHASE_WIDTH = %d\n",(unsigned int)TRACK_INIT_CODE_PHASE_WIDTH); for (u64 i=0; i<1000; i++) __asm__("nop");
  printf("TRACK_CARRIER_FREQ_WIDTH = %d\n",(unsigned int)TRACK_CARRIER_FREQ_WIDTH); for (u64 i=0; i<1000; i++) __asm__("nop");
  printf("TRACK_CODE_PHASE_FRACTIONAL_WIDTH = %d\n",(unsigned int)TRACK_CODE_PHASE_FRACTIONAL_WIDTH); for (u64 i=0; i<1000; i++) __asm__("nop");
  printf("TRACK_CODE_PHASE_RATE_WIDTH = %d\n",(unsigned int)TRACK_CODE_PHASE_RATE_WIDTH); for (u64 i=0; i<1000; i++) __asm__("nop");
  printf("CW_CARRIER_FREQ_WIDTH = %d\n",(unsigned int)CW_CARRIER_FREQ_WIDTH); for (u64 i=0; i<1000; i++) __asm__("nop");
//  printf("IIR_COEF_LSB = %d\n",(unsigned int)IIR_COEF_LSB); for (u64 i=0; i<1000; i++) __asm__("nop");
//  printf("IIR_COEF_MSB = %d\n",(unsigned int)IIR_COEF_MSB); for (u64 i=0; i<1000; i++) __asm__("nop");
//  printf("IIR_N_TAPS = %d\n",(unsigned int)IIR_N_TAPS); for (u64 i=0; i<1000; i++) __asm__("nop");
  printf("TRACK_N_CHANNELS = %d\n",(unsigned int)TRACK_N_CHANNELS); for (u64 i=0; i<1000; i++) __asm__("nop");

  printf("ACQ_CODE_PHASE_UNITS_PER_CHIP = %ld\n",(long int)ACQ_CODE_PHASE_UNITS_PER_CHIP); for (u64 i=0; i<1000; i++) __asm__("nop");
  printf("ACQ_CARRIER_FREQ_UNITS_PER_HZ = %10.5f\n",ACQ_CARRIER_FREQ_UNITS_PER_HZ); for (u64 i=0; i<1000; i++) __asm__("nop");
  printf("TRACK_INIT_CODE_PHASE_UNITS_PER_CHIP = %ld\n",(long int)TRACK_INIT_CODE_PHASE_UNITS_PER_CHIP); for (u64 i=0; i<1000; i++) __asm__("nop");
  printf("TRACK_CARRIER_FREQ_UNITS_PER_HZ = %10.5f\n",TRACK_CARRIER_FREQ_UNITS_PER_HZ); for (u64 i=0; i<1000; i++) __asm__("nop");
  printf("TRACK_NOMINAL_CODE_PHASE_RATE = %ld\n",(long int)TRACK_NOMINAL_CODE_PHASE_RATE); for (u64 i=0; i<1000; i++) __asm__("nop");
  printf("TRACK_CODE_PHASE_RATE_UNITS_PER_HZ = %10.5f\n",TRACK_CODE_PHASE_RATE_UNITS_PER_HZ); for (u64 i=0; i<1000; i++) __asm__("nop");
  printf("TRACK_CODE_PHASE_UNITS_PER_CHIP = %ld\n",(long int)TRACK_CODE_PHASE_UNITS_PER_CHIP); for (u64 i=0; i<1000; i++) __asm__("nop");
  printf("CW_CARRIER_FREQ_UNITS_PER_HZ = %10.5f\n",CW_CARRIER_FREQ_UNITS_PER_HZ); for (u64 i=0; i<1000; i++) __asm__("nop");

  u8 git_hash[20];
  get_nap_git_hash(git_hash);
  printf("git_hash = 0x");
  for (u8 i=0;i<20;i++){
    printf("%x",(unsigned int)git_hash[i]);
  }
  printf("\n");
  u8 git_unclean = get_nap_git_unclean();
  printf("git_unclean = %d\n",(unsigned int)git_unclean);

///* Constants derived from NAP Parameters stored in the configuration flash */
//extern u64 ACQ_CODE_PHASE_UNITS_PER_CHIP;
//extern float ACQ_CARRIER_FREQ_UNITS_PER_HZ;
//extern u8 TRACK_INIT_CODE_PHASE_UNITS_PER_CHIP;
//extern float TRACK_CARRIER_FREQ_UNITS_PER_HZ;
//extern u32 TRACK_NOMINAL_CODE_PHASE_RATE;
//extern float TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;
//extern u64 TRACK_CODE_PHASE_UNITS_PER_CHIP;
//extern float CW_CARRIER_FREQ_UNITS_PER_HZ;

  float code_phase;
  float carrier_freq;
  float snr;

  acq_schedule_load(timing_count() + 2000);
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

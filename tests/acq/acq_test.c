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

#include "init.h"
#include "main.h"
#include "sbp.h"
#include "acq.h"
#include "board/leds.h"
#include "board/m25_flash.h"
#include "board/nap/acq_channel.h"

int main(void)
{

  init(1);

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n\r");
  printf("--- ACQ TEST ---\n\r");

  float code_phase;
  float carrier_freq;
  float snr;

  acq_schedule_load(nap_timing_count() + 1000);
  while(!(acq_get_load_done()));
  led_off(LED_GREEN);
  led_off(LED_RED);

  for (u8 prn=0; prn<32; prn++) {
    nap_acq_code_wr_blocking(prn);
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


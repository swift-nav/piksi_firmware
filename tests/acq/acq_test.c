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
#include "board/nap/nap_common.h"

/** Send results of an acquisition to the host.
 *
 * \param sv  PRN (0-31) of the acquisition
 * \param snr Signal to noise ratio of best point from acquisition.
 * \param cp  Code phase of best point.
 * \param cf  Carrier frequency of best point.
 * \param bc  Correlation of best point.
 * \param mc  Mean correlation of all points.
 * \return    Return value from sending message.
 */
//u32 acq_send_result(u8 sv, float snr, float cp, float cf, corr_t bc, u32 mc)
u32 acq_send_result(u8 sv, float snr, float cp, float cf)
{
  typedef struct __attribute__((packed)) {
    u8 sv;
    float snr;
    float cp;
    float cf;
    corr_t bc;
    u32 mc;
  } acq_result_msg_t;

  acq_result_msg_t acq_result_msg;

  acq_result_msg.sv = sv;
  acq_result_msg.snr = snr;
  acq_result_msg.cp = cp;
  acq_result_msg.cf = cf;
//  acq_result_msg.bc = bc;
  acq_result_msg.bc.I = 0;
  acq_result_msg.bc.Q = 0;
  acq_result_msg.mc = 0;

  /* Keep trying to send message until it makes it into the buffer. */
  while (sbp_send_msg(MSG_ACQ_RESULT, sizeof(acq_result_msg_t), \
                      (u8 *)&acq_result_msg)) ;

  return 0;
}

int main(void)
{
  init();

  for (u32 i=0; i<100000; i++)
    __asm__("NOP");

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n\r");
  printf("--- ACQ TEST ---\n\r");

  float code_phase;
  float carrier_freq;
  float snr;
//  corr_t best_corr;
//  u32 mean_corr;

  u8 prn = 0;
  while (1) {
    acq_schedule_load(nap_timing_count() + 1000);
    while(!(acq_get_load_done()));

    nap_acq_code_wr_blocking(prn);
    acq_start(prn, 0, 1023, -7000, 7000, 300);
    while(!(acq_get_done()));

    acq_get_results(&code_phase, &carrier_freq, &snr);
    acq_send_result(prn, snr, code_phase, carrier_freq);

    //printf("PRN %2u - CP: %7.2f, CF: % 7.1f, SNR: %5.2f, Max I: %6d, Max Q: %6d, Mean Corr: %5d", prn+1, code_phase, carrier_freq, snr, (unsigned int)best_corr.I, (unsigned int)best_corr.Q, (unsigned int)mean_corr);
    printf("PRN %2u - CP: %7.2f, CF: % 7.1f, SNR: %5.2f", prn+1, code_phase, carrier_freq, snr);
    if (snr > 25.0)
      printf("   :D\n");
    else
      printf("\n");
    led_toggle(LED_GREEN);
    led_toggle(LED_RED);

    if (prn == 31) {
      prn = 0;
    } else {
      prn++;
    }

    sbp_process_messages();
  }

  printf("DONE!\n");
  led_on(LED_GREEN);
  led_off(LED_RED);
  while (1);

	return 0;
}


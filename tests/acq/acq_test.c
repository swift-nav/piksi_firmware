/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdio.h>

#include "init.h"
#include "main.h"
#include "sbp.h"
#include "acq.h"
#include "board/leds.h"
#include "board/m25_flash.h"
#include "board/nap/acq_channel.h"

/** Send results of an acquisition to the host.
 *
 * \param sv  PRN (0-31) of the acquisition
 * \param snr Signal to noise ratio of best point from acquisition.
 * \param cp  Code phase of best point.
 * \param cf  Carrier frequency of best point.
 */
void acq_send_result(u8 sv, float snr, float cp, float cf)
{
  typedef struct __attribute__((packed)) {
    float snr; /* SNR of best point. */
    float cp;  /* Code phase of best point. */
    float cf;  /* Carr freq of best point. */
    corr_t bc; /* Correlations of best point. */
    u32 mc;    /* Mean correlation. */
    u8 sv;     /* SV searched for. */
  } acq_result_msg_t;

  acq_result_msg_t acq_result_msg;

  acq_result_msg.sv = sv;
  acq_result_msg.snr = snr;
  acq_result_msg.cp = cp;
  acq_result_msg.cf = cf;
  acq_result_msg.bc.I = 0; /* Currently unused. */
  acq_result_msg.bc.Q = 0; /* Currently unused. */
  acq_result_msg.mc = 0;   /* Currently unused. */

  sbp_send_msg(MSG_ACQ_RESULT,
               sizeof(acq_result_msg_t),
               (u8 *)&acq_result_msg);
}

int main(void)
{

  init();

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n\r");
  printf("--- ACQ TEST ---\n\r");

  float code_phase;
  float carrier_freq;
  float snr;

  u8 prn = 0;
  while (1) {
    acq_schedule_load(nap_timing_count() + 1000);
    while(!(acq_get_load_done()));

    nap_acq_code_wr_blocking(prn);
    acq_start(prn, 0, 1023, -7000, 7000, 300);
    while(!(acq_get_done()));

    acq_get_results(&code_phase, &carrier_freq, &snr);
    acq_send_result(prn, snr, code_phase, carrier_freq);

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


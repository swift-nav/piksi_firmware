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
#include <math.h>

#include "init.h"
#include "main.h"
#include "sbp.h"
#include "track.h"
#include "acq.h"
#include "board/leds.h"
#include "board/m25_flash.h"
#include "board/nap/nap_exti.h"
#include "board/nap/acq_channel.h"
#include "board/nap/track_channel.h"

int main(void)
{
  init();

  #ifndef PRN
    #error Please define the PRN to be used, e.g. make PRN=22
  #endif

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");
  printf("--- CODE PHASE PROPAGATION TEST ---\n");

  while(1) {
    printf("\nPRN: %u\n", PRN);

    /* Initial coarse acq. */
    float coarse_acq_code_phase;
    float coarse_acq_carrier_freq;
    float coarse_snr;
    nap_acq_load_wr_enable_blocking();
    u32 coarse_acq_cnt = nap_timing_count() + 1000;
    nap_timing_strobe(coarse_acq_cnt);
    wait_for_nap_exti();
    nap_acq_load_wr_disable_blocking();

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
    nap_acq_load_wr_enable_blocking();
    u32 fine_acq_cnt = nap_timing_count() + 2000;
    nap_timing_strobe(fine_acq_cnt);
    wait_for_nap_exti();
    nap_acq_load_wr_disable_blocking();

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
    nap_acq_load_wr_enable_blocking();
    u32 fine2_acq_cnt = nap_timing_count() + 2000;
    nap_timing_strobe(fine2_acq_cnt);
    wait_for_nap_exti();
    nap_acq_load_wr_disable_blocking();

    float fine2_cp = propagate_code_phase(fine_acq_code_phase, fine_acq_carrier_freq, fine2_acq_cnt - fine_acq_cnt);

    do_acq(PRN-1, fine2_cp-20, fine2_cp+20, fine_acq_carrier_freq-300, fine_acq_carrier_freq+300, 100, &fine2_acq_code_phase, &fine2_acq_carrier_freq, &fine2_snr);

    dt = (float)(fine2_acq_cnt - fine_acq_cnt) * 1000.0 / SAMPLE_FREQ;
    printf("Second fine:\n  Propagated for %.1f ms, propagated code phase %.4f\n", dt, fine2_cp);
    printf("  Code phase: %7.4f, Carrier freq % 7.1f, SNR %5.2f\n", fine2_acq_code_phase, fine2_acq_carrier_freq, fine2_snr);
    printf("  Code phase error: %f\n", fine2_acq_code_phase - fine2_cp);
  }

	return 0;
}


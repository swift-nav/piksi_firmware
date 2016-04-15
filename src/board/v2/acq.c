/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "acq.h"

#include "nap/nap_acq.h"
#include "nap/acq_channel.h"
#include "nap_common.h"

float acq_bin_width(void)
{
  return (float)NAP_ACQ_SAMPLE_FREQ / (1 << NAP_ACQ_CARRIER_FREQ_WIDTH);
}

bool acq_search(gnss_signal_t sid, float cf_min, float cf_max,
                float cf_bin_width, acq_result_t *acq_result)
{
  acq_set_sid(sid);

  /* We have our SID chosen, now load some fresh data
   * into the acquisition ram on the Swift NAP for
   * an initial coarse acquisition.
   */
  u32 sample_count;
  do {
    sample_count = nap_timing_count() + 20000;
    /* acq_load could timeout if we're preempted and miss the timing strobe */
  } while (!acq_load(sample_count));

  acq_search_begin(cf_min, cf_max, cf_bin_width);

  /* Done with the coarse acquisition, check if we have found a
   * satellite, if so save the results and start the loading
   * for the fine acquisition. If not, start again choosing a
   * different PRN.
   */
  acq_result->sample_count = sample_count;
  acq_get_results(&acq_result->cp, &acq_result->cf, &acq_result->cn0);
  return true;
}

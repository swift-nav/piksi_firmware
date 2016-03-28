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

bool acq_search(gnss_signal_t sid, float cf_min, float cf_max,
                float cf_bin_width, acq_result_t *acq_result)
{
  (void)sid;
  (void)cf_min;
  (void)cf_max;
  (void)cf_bin_width;
  (void)acq_result;
  return false;
}

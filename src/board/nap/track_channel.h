/*
 * Copyright (C) 2011-2014,2016 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_TRACK_CHANNEL_H
#define SWIFTNAV_TRACK_CHANNEL_H

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>

#include "nap_common.h"

/** \addtogroup track_channel
 * \{ */

extern u8 nap_track_n_channels;

/** \} */

void nap_track_init(u8 channel, gnss_signal_t sid, u32 ref_timing_count,
                    float carrier_freq, float code_phase, u32 chips_to_correlate);

void nap_track_update(u8 channel, double carrier_freq,
                      double code_phase_rate, u32 chips_to_correlate,
                      u8 corr_spacing);
void nap_track_read_results(u8 channel,
                            u32* count_snapshot, corr_t corrs[],
                            double *code_phase_early,
                            double *carrier_phase);

void nap_track_disable(u8 channel);

#endif  /* SWIFTNAV_TRACK_CHANNEL_H */


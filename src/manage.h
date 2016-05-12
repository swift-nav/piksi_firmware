/*
 * Copyright (C) 2011-2014,2016 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_MANAGE_H
#define SWIFTNAV_MANAGE_H

#include <ch.h>
#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>
#include "board/acq.h"

/** \addtogroup manage
 * \{ */

#define ACQ_THRESHOLD 37.0
#define ACQ_RETRY_THRESHOLD 38.0

#define TRACK_SNR_THRES_COUNT 2000

/** How many ms to allow tracking channel to converge after
    initialization before we consider dropping it */
#define TRACK_INIT_T 2500

/** If a channel is dropped but was running successfully for at least
    this long, mark it for prioritized reacquisition. */
#define TRACK_REACQ_T 5000

/** If C/N0 is below track_cn0_threshold for >= TRACK_DROP_CN0_T ms,
    drop the channel. */
#define TRACK_DROP_CN0_T 5000

/** If optimistic phase lock detector shows "unlocked" for >=
    TRACK_DROP_UNLOCKED_T ms, drop the channel. */
#define TRACK_DROP_UNLOCKED_T 5000

/** If pessimistic phase lock detector shows "locked" for >=
    TRACK_USE_LOCKED_T ms, use the channel. */
#define TRACK_USE_LOCKED_T 100

/** How many milliseconds to wait for the tracking loops to
 * stabilize after any mode change before using obs. */
#define TRACK_STABILIZATION_T 1000

#define ACQ_FULL_CF_MIN  -8500
#define ACQ_FULL_CF_MAX   8500
#define ACQ_FULL_CF_STEP  acq_bin_width()

#define MANAGE_NO_CHANNELS_FREE 255

#define MANAGE_ACQ_THREAD_PRIORITY (NORMALPRIO-3)
#define MANAGE_ACQ_THREAD_STACK    1400

#define MANAGE_TRACK_THREAD_PRIORITY (NORMALPRIO-2)
#define MANAGE_TRACK_THREAD_STACK   1400

typedef struct {
  gnss_signal_t sid;      /**< Signal identifier. */
  u32 sample_count;       /**< Reference NAP sample count. */
  float carrier_freq;     /**< Carrier frequency Doppler (Hz). */
  float code_phase;       /**< Code phase (chips). */
  u32 chips_to_correlate; /**< Chips to integrate over. */
  float cn0_init;         /**< C/N0 estimate (dBHz). */
  s8 elevation;           /**< Elevation (deg). */
} tracking_startup_params_t;

/** \} */

void manage_acq_setup(void);

void manage_set_obs_hint(gnss_signal_t sid);

void manage_track_setup(void);
s8 use_tracking_channel(u8 i);
u8 tracking_channels_ready(void);

bool tracking_startup_request(const tracking_startup_params_t *startup_params);

#endif

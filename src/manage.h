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

#ifndef SWIFTNAV_MANAGE_H
#define SWIFTNAV_MANAGE_H

#include <ch.h>
#include <libswiftnav/common.h>
#include "board/nap/acq_channel.h"

/** \addtogroup manage
 * \{ */

#define ACQ_THRESHOLD 37.0
#define ACQ_RETRY_THRESHOLD 38.0

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

/** How many milliseconds to wait for the tracking loops to
 * stabilize after any mode change before using obs. */
#define TRACK_STABILIZATION_T 1000

#define ACQ_FULL_CF_MIN  -8500
#define ACQ_FULL_CF_MAX   8500
#define ACQ_FULL_CF_STEP  (1 / NAP_ACQ_CARRIER_FREQ_UNITS_PER_HZ)

#define MANAGE_NO_CHANNELS_FREE 255

#define MANAGE_ACQ_THREAD_PRIORITY (NORMALPRIO-3)
#define MANAGE_ACQ_THREAD_STACK    1400

#define MANAGE_TRACK_THREAD_PRIORITY (NORMALPRIO-2)
#define MANAGE_TRACK_THREAD_STACK   1400

/** \} */

void manage_acq_setup(void);

void manage_set_obs_hint(u16 prn);

void manage_track_setup(void);
s8 use_tracking_channel(u8 i);
u8 tracking_channels_ready(void);

#endif

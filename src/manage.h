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

/** \addtogroup manage
 * \{ */

#define ACQ_THRESHOLD 20.0
#define TRACK_THRESHOLD 2.0
#define TRACK_SNR_INIT_COUNT 5000
#define TRACK_SNR_THRES_COUNT 2000

#define ACQ_FULL_CF_MIN  -8500
#define ACQ_FULL_CF_MAX   8500
#define ACQ_FULL_CF_STEP  400
#define ACQ_FINE_CF_WIDTH 500
#define ACQ_FINE_CP_WIDTH 20
#define ACQ_FINE_CF_STEP  50

#define MANAGE_NO_CHANNELS_FREE 255

#define MANAGE_ACQ_THREAD_PRIORITY (NORMALPRIO-3)
#define MANAGE_ACQ_THREAD_STACK    3000

#define MANAGE_TRACK_THREAD_PRIORITY (NORMALPRIO-2)
#define MANAGE_TRACK_THREAD_STACK    3000

/** Acquisition management states. */
typedef enum {
  ACQ_MANAGE_START = 0,
  ACQ_MANAGE_DISABLED,
  ACQ_MANAGE_LOADING_COARSE,
  ACQ_MANAGE_RUNNING_COARSE,
  ACQ_MANAGE_LOADING_FINE,
  ACQ_MANAGE_RUNNING_FINE
} acq_manage_state_t;

/** Status of acquisition for a particular PRN. */
typedef struct __attribute__((packed)) {
  enum {
    ACQ_PRN_SKIP = 0,
    ACQ_PRN_UNTRIED,
    ACQ_PRN_TRIED,
    ACQ_PRN_ACQUIRING,
    ACQ_PRN_TRACKING
  } state;  /**< Management status of PRN. */
  s8 score; /**< Acquisition preference of PRN. */
} acq_prn_t;

/** \} */

void manage_acq_setup(void);
void manage_acq(void);

void manage_prod_acq(u8 prn);

void manage_track_setup(void);
u8 manage_track_new_acq(float snr);
void manage_track(void);
s8 use_tracking_channel(u8 i);
u8 tracking_channels_ready(void);

#endif

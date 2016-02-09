/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_BASE_OBS_H
#define SWIFTNAV_BASE_OBS_H

#include <ch.h>

#include <libswiftnav/common.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/track.h>
#include <libswiftnav/time.h>

/** \addtogroup base_obs Base station observation handling
 * \{ */

typedef struct {
  /** GPS time of the observation. */
  gps_time_t t;
  /** Approximate base station position.
   * This may be the position as reported by the base station itself or the
   * position obtained from doing a single point solution using the base
   * station observations. */
  double pos_ecef[3];
  /** Is the `pos_ecef` field valid? */
  u8 has_pos;
  /** Number of observations in the set. */
  u8 n;
  /** Set of observations. */
  navigation_measurement_t nm[MAX_CHANNELS];
  /** Distances to each satellite based on `pos_ecef` and `nm`.
   * Used for observation propagation. */
  double sat_dists[MAX_CHANNELS];
} obss_t;

/** Maximum difference between observation times to consider them matched. */
#define TIME_MATCH_THRESHOLD 2e-3

/* \} */

extern mutex_t base_obs_lock;
extern binary_semaphore_t base_obs_received;
extern obss_t base_obss;

extern mutex_t base_pos_lock;
extern bool base_pos_known;
extern double base_pos_ecef[3];

void base_obs_setup(void);

#endif


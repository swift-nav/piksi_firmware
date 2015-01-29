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
#include <libswiftnav/gpstime.h>

typedef struct {
  gps_time_t t;
  double pos_ecef[3];
  u8 has_pos;
  u8 n;
  navigation_measurement_t nm[MAX_CHANNELS];
  double sat_dists[MAX_CHANNELS];
} obss_t;

extern Mutex base_obs_lock;
extern BinarySemaphore base_obs_received;
extern obss_t base_obss;

extern Mutex base_pos_lock;
extern bool_t base_pos_known;
extern double base_pos_ecef[3];

void base_obs_setup(void);

#endif


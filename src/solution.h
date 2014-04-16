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

#ifndef SWIFTNAV_SOLUTION_H
#define SWIFTNAV_SOLUTION_H

#include <libswiftnav/common.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/track.h>
#include <libswiftnav/gpstime.h>

typedef struct {
  void *next; /* Used by memory pool implementation. */
  gps_time_t t;
  u8 n;
  navigation_measurement_t nm[MAX_CHANNELS];
} obss_t;

typedef enum {
  SOLN_MODE_LOW_LATENCY,
  SOLN_MODE_TIME_MATCHED
} dgnss_solution_mode_t;

typedef enum {
  FILTER_FLOAT,
  FILTER_FIXED,
} dgnss_filter_t;

/** Maximum difference between observation times to consider them matched. */
#define TIME_MATCH_THRESHOLD 1e-6
/** Maximum time that an observation will be propagated for to align it with a
 * solution epoch before it is discarded.  */
#define OBS_PROPAGATION_LIMIT 10e-3

#define MAX_AGE_OF_DIFFERENTIAL 1.0

#define SOLN_FREQ 3.0

#define OBS_N_BUFF 5
#define OBS_BUFF_SIZE (OBS_N_BUFF * sizeof(obss_t))

void solution_send_sbp(gnss_solution *soln, dops_t *dops);
void solution_send_nmea(gnss_solution *soln, dops_t *dops,
                        u8 n, navigation_measurement_t *nm);
void solution_send_baseline(gps_time_t *t, u8 n_sats, double b_ecef[3],
                            double ref_ecef[3], u8 flags);
void solution_setup(void);

#endif


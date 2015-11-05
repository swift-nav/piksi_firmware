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

#include <ch.h>
#include <libswiftnav/common.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/track.h>
#include <libswiftnav/gpstime.h>
#include <libswiftnav/dgnss_management.h>

typedef enum {
  SOLN_MODE_LOW_LATENCY,
  SOLN_MODE_TIME_MATCHED
} dgnss_solution_mode_t;

typedef enum {
  FILTER_FLOAT,
  FILTER_FIXED,
} dgnss_filter_t;

/** Maximum time that an observation will be propagated for to align it with a
 * solution epoch before it is discarded.  */
#define OBS_PROPAGATION_LIMIT 10e-3

#define MAX_AGE_OF_DIFFERENTIAL 1.0

#define DGNSS_TIMEOUT S2ST(2)

#define OBS_N_BUFF 5
#define OBS_BUFF_SIZE (OBS_N_BUFF * sizeof(obss_t))

extern double soln_freq;
extern u32 obs_output_divisor;

void solution_send_sbp(gnss_solution *soln, dops_t *dops);
void solution_send_nmea(gnss_solution *soln, dops_t *dops,
                        u8 n, navigation_measurement_t *nm,
                        u8 fix_type);
double calc_heading(const double b_ned[3]);
void solution_send_baseline(const gps_time_t *t, dgnss_baseline_t *solution,
                            double ref_ecef[3]);
void solution_setup(void);

#endif


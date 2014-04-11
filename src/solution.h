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
#include <libswiftnav/pvt.h>
#include <libswiftnav/track.h>

#define MAX_SATS 14
#define MAX_CHANNELS 14

void solution_send_sbp(gnss_solution *soln, dops_t *dops);
void solution_send_nmea(gnss_solution *soln, dops_t *dops,
                        u8 n, navigation_measurement_t *nm);
void solution_send_baseline(gps_time_t *t, u8 n_sats, double b_ecef[3],
                            double ref_ecef[3]);
void solution_setup(void);

#endif


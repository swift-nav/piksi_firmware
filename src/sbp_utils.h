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

#ifndef SWIFTNAV_SBP_UTILS_H
#define SWIFTNAV_SBP_UTILS_H

#include <libsbp/common.h>
#include <libsbp/sbp_messages.h>
#include <libswiftnav/gpstime.h>
#include <libswiftnav/pvt.h>

void sbp_make_gps_time(sbp_gps_time_t *t_out, gps_time_t *t_in, u8 flags);
void sbp_make_pos_llh(sbp_pos_llh_t *pos_llh, gnss_solution *soln, u8 flags);
void sbp_make_pos_llh_vect(sbp_pos_llh_t *pos_llh, double llh[3],
                           gps_time_t *gps_t, u8 n_used, u8 flags);
void sbp_make_pos_ecef(sbp_pos_ecef_t *pos_ecef, gnss_solution *soln, u8 flags);
void sbp_make_pos_ecef_vect(sbp_pos_ecef_t *pos_ecef, double ecef[3],
                            gps_time_t *gps_t, u8 n_used, u8 flags);
void sbp_make_vel_ned(sbp_vel_ned_t *vel_ned, gnss_solution *soln, u8 flags);
void sbp_make_vel_ecef(sbp_vel_ecef_t *vel_ecef, gnss_solution *soln, u8 flags);
void sbp_make_dops(sbp_dops_t *dops_out, dops_t *dops_in);
void sbp_make_baseline_ecef(sbp_baseline_ecef_t *baseline_ecef, gps_time_t *t,
                            u8 n_sats, double b_ecef[3], u8 flags);
void sbp_make_baseline_ned(sbp_baseline_ned_t *baseline_ned, gps_time_t *t,
                           u8 n_sats, double b_ned[3], u8 flags);

#endif /* SWIFTNAV_SBP_UTILS_H */

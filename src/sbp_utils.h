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
#include <libsbp/navigation.h>
#include <libsbp/observation.h>
#include <libsbp/gnss_signal.h>
#include <libswiftnav/time.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/signal.h>

void sbp_make_gps_time(msg_gps_time_t *t_out, const gps_time_t *t_in, u8 flags);
void sbp_make_pos_llh(msg_pos_llh_t *pos_llh, const gnss_solution *soln, u8 flags);
void sbp_make_pos_llh_vect(msg_pos_llh_t *pos_llh, const double llh[3],
                           const gps_time_t *gps_t, u8 n_used, u8 flags);
void sbp_make_pos_ecef(msg_pos_ecef_t *pos_ecef, const gnss_solution *soln, u8 flags);
void sbp_make_pos_ecef_vect(msg_pos_ecef_t *pos_ecef, const double ecef[3],
                            const gps_time_t *gps_t, u8 n_used, u8 flags);
void sbp_make_vel_ned(msg_vel_ned_t *vel_ned, const gnss_solution *soln, u8 flags);
void sbp_make_vel_ecef(msg_vel_ecef_t *vel_ecef, const gnss_solution *soln, u8 flags);
void sbp_make_dops(msg_dops_t *dops_out, const dops_t *dops_in, const gps_time_t *t);
void sbp_make_baseline_ecef(msg_baseline_ecef_t *baseline_ecef, const gps_time_t *t,
                            u8 n_sats, const double b_ecef[3], u8 flags);
void sbp_make_baseline_ned(msg_baseline_ned_t *baseline_ned, const gps_time_t *t,
                           u8 n_sats, const double b_ned[3], u8 flags);
void sbp_make_heading(msg_baseline_heading_t *baseline_heading, const gps_time_t *t,
                      const double heading, u8 n_used, u8 flags);
#define MSG_OBS_HEADER_SEQ_SHIFT 4u
#define MSG_OBS_HEADER_SEQ_MASK ((1 << 4u) - 1)
#define MSG_OBS_HEADER_MAX_SIZE MSG_OBS_HEADER_SEQ_MASK
#define MSG_OBS_TOW_MULTIPLIER ((double)1000.0)

#define MSG_OBS_P_MULTIPLIER ((double)1e2)
#define MSG_OBS_SNR_MULTIPLIER ((float)4)
#define MSG_OSB_LF_MULTIPLIER ((double)(1<<8))

void unpack_obs_header(const observation_header_t *msg, gps_time_t* t,
                       u8* total, u8* count);

void pack_obs_header(const gps_time_t *t, u8 total, u8 count,
                     observation_header_t *msg);

void unpack_obs_content(const packed_obs_content_t *msg, double *P, double *L,
                        double *snr, u16 *lock_counter, gnss_signal_t *sid);

s8 pack_obs_content(double P, double L, double snr, u16 lock_counter,
                    gnss_signal_t sid, packed_obs_content_t *msg);

void unpack_ephemeris(const msg_ephemeris_t *msg, ephemeris_t *e);

void pack_ephemeris(const ephemeris_t *e, msg_ephemeris_t *msg);

/** Value specifying the size of the SBP framing */
#define SBP_FRAMING_SIZE_BYTES 8
/** Value defining maximum SBP packet size */
#define SBP_FRAMING_MAX_PAYLOAD_SIZE 255

gnss_signal_t sid_from_sbp(const sbp_gnss_signal_t from);
sbp_gnss_signal_t sid_to_sbp(const gnss_signal_t from);

#endif /* SWIFTNAV_SBP_UTILS_H */

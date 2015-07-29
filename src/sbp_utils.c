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

#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>

#include "chdebug.h"
#include "sbp_utils.h"

/** \addtogroup sbp
 * \{ */

/** \defgroup sbp_utils SBP Utils
 * Convert to and from SBP message types and other useful functions.
 * \{ */

void sbp_make_gps_time(msg_gps_time_t *t_out, const gps_time_t *t_in, u8 flags)
{
  t_out->wn = t_in->wn;
  t_out->tow = round(t_in->tow * 1e3);
  t_out->ns = round((t_in->tow - t_out->tow*1e-3) * 1e9);
  t_out->flags = flags;
}

void sbp_make_pos_llh(msg_pos_llh_t *pos_llh, const gnss_solution *soln, u8 flags)
{
  pos_llh->tow = round(soln->time.tow * 1e3);
  pos_llh->lat = soln->pos_llh[0] * R2D;
  pos_llh->lon = soln->pos_llh[1] * R2D;
  pos_llh->height = soln->pos_llh[2];
  /* TODO: fill in accuracy fields. */
  pos_llh->h_accuracy = 0;
  pos_llh->v_accuracy = 0;
  pos_llh->n_sats = soln->n_used;
  pos_llh->flags = flags;
}

void sbp_make_pos_llh_vect(msg_pos_llh_t *pos_llh, const double llh[3],
                           const gps_time_t *gps_t, u8 n_used, u8 flags)
{
  pos_llh->tow = round(gps_t->tow * 1e3);
  pos_llh->lat = llh[0] * R2D;
  pos_llh->lon = llh[1] * R2D;
  pos_llh->height = llh[2];
  /* TODO: fill in accuracy field. */
  pos_llh->h_accuracy = 0;
  pos_llh->v_accuracy = 0;
  pos_llh->n_sats = n_used;
  pos_llh->flags = flags;
}

void sbp_make_pos_ecef(msg_pos_ecef_t *pos_ecef, const gnss_solution *soln, u8 flags)
{
  pos_ecef->tow = round(soln->time.tow * 1e3);
  pos_ecef->x = soln->pos_ecef[0];
  pos_ecef->y = soln->pos_ecef[1];
  pos_ecef->z = soln->pos_ecef[2];
  /* TODO: fill in accuracy field. */
  pos_ecef->accuracy = 0;
  pos_ecef->n_sats = soln->n_used;
  pos_ecef->flags = flags;
}

void sbp_make_pos_ecef_vect(msg_pos_ecef_t *pos_ecef, const double ecef[3],
                            const gps_time_t *gps_t, u8 n_used, u8 flags)
{
  pos_ecef->tow = round(gps_t->tow * 1e3);
  pos_ecef->x = ecef[0];
  pos_ecef->y = ecef[1];
  pos_ecef->z = ecef[2];
  /* TODO: fill in accuracy field. */
  pos_ecef->accuracy = 0;
  pos_ecef->n_sats = n_used;
  pos_ecef->flags = flags;
}

void sbp_make_vel_ned(msg_vel_ned_t *vel_ned, const gnss_solution *soln, u8 flags)
{
  vel_ned->tow = round(soln->time.tow * 1e3);
  vel_ned->n = round(soln->vel_ned[0] * 1e3);
  vel_ned->e = round(soln->vel_ned[1] * 1e3);
  vel_ned->d = round(soln->vel_ned[2] * 1e3);
  /* TODO: fill in accuracy fields. */
  vel_ned->h_accuracy = 0;
  vel_ned->v_accuracy = 0;
  vel_ned->n_sats = soln->n_used;
  vel_ned->flags = flags;
}

void sbp_make_vel_ecef(msg_vel_ecef_t *vel_ecef, const gnss_solution *soln, u8 flags)
{
  vel_ecef->tow = round(soln->time.tow * 1e3);
  vel_ecef->x = round(soln->vel_ecef[0] * 1e3);
  vel_ecef->y = round(soln->vel_ecef[1] * 1e3);
  vel_ecef->z = round(soln->vel_ecef[2] * 1e3);
  /* TODO: fill in accuracy field. */
  vel_ecef->accuracy = 0;
  vel_ecef->n_sats = soln->n_used;
  vel_ecef->flags = flags;
}

void sbp_make_dops(msg_dops_t *dops_out, const dops_t *dops_in, const gps_time_t *t)
{
  dops_out->tow = round(t->tow * 1e3);
  dops_out->pdop = round(dops_in->pdop * 100);
  dops_out->gdop = round(dops_in->gdop * 100);
  dops_out->tdop = round(dops_in->tdop * 100);
  dops_out->hdop = round(dops_in->hdop * 100);
  dops_out->vdop = round(dops_in->vdop * 100);
}

void sbp_make_baseline_ecef(msg_baseline_ecef_t *baseline_ecef, const gps_time_t *t,
                            u8 n_sats, const double b_ecef[3], u8 flags) {
  baseline_ecef->tow = round(t->tow * 1e3);
  baseline_ecef->x = round(1e3 * b_ecef[0]);
  baseline_ecef->y = round(1e3 * b_ecef[1]);
  baseline_ecef->z = round(1e3 * b_ecef[2]);
  baseline_ecef->accuracy = 0;
  baseline_ecef->n_sats = n_sats;
  baseline_ecef->flags = flags;
}

void sbp_make_baseline_ned(msg_baseline_ned_t *baseline_ned, const gps_time_t *t,
                           u8 n_sats, const double b_ned[3], u8 flags) {
  baseline_ned->tow = round(t->tow * 1e3);
  baseline_ned->n = round(1e3 * b_ned[0]);
  baseline_ned->e = round(1e3 * b_ned[1]);
  baseline_ned->d = round(1e3 * b_ned[2]);
  baseline_ned->h_accuracy = 0;
  baseline_ned->v_accuracy = 0;
  baseline_ned->n_sats = n_sats;
  baseline_ned->flags = flags;
}

void unpack_obs_header(const observation_header_t *msg, gps_time_t* t, u8* total, u8* count)
{
  t->tow = ((double)msg->t.tow) / MSG_OBS_TOW_MULTIPLIER;
  t->wn  = msg->t.wn;
  *total = (msg->n_obs >> MSG_OBS_HEADER_SEQ_SHIFT);
  *count = (msg->n_obs & MSG_OBS_HEADER_SEQ_MASK);
}

void pack_obs_header(const gps_time_t *t, u8 total, u8 count, observation_header_t *msg)
{
  msg->t.tow = (u32)round(t->tow * MSG_OBS_TOW_MULTIPLIER);
  msg->t.wn  = t->wn;
  msg->n_obs = ((total << MSG_OBS_HEADER_SEQ_SHIFT) |
                 (count & MSG_OBS_HEADER_SEQ_MASK));
}

void unpack_obs_content(const packed_obs_content_t *msg, double *P, double *L,
                        double *snr, u16 *lock_counter, u8 *prn)
{
  *P   = ((double)msg->P) / MSG_OBS_P_MULTIPLIER;
  *L   = ((double)msg->L.i) + (((double)msg->L.f) / MSG_OSB_LF_MULTIPLIER);
  *snr = ((double)msg->cn0) / MSG_OBS_SNR_MULTIPLIER;
  *lock_counter = ((u16)msg->lock);
  *prn = msg->sid & 0x1F; /* TODO: prn -> sid */
}

/** Pack GPS observables into a `msg_obs_content_t` struct.
 * For use in constructing a `MSG_NEW_OBS` SBP message.
 *
 * \param P Pseudorange in meters
 * \param L Carrier phase in cycles
 * \param snr Signal-to-noise ratio
 * \param lock_counter Lock counter is an arbitrary integer that should change
 *                     if the carrier phase ambiguity is ever reset
 * \param prn Satellite PRN identifier
 * \param msg Pointer to a `msg_obs_content_t` struct to fill out
 * \return `0` on success or `-1` on an overflow error
 */
s8 pack_obs_content(double P, double L, double snr, u16 lock_counter, u8 prn,
                    packed_obs_content_t *msg)
{

  s64 P_fp = llround(P * MSG_OBS_P_MULTIPLIER);
  if (P < 0 || P_fp > UINT32_MAX) {
    log_error("observation message packing: P integer overflow (%f)", P);
    return -1;
  }

  msg->P = (u32)P_fp;

  double Li = floor(L);
  if (Li < INT32_MIN || Li > INT32_MAX) {
    log_error("observation message packing: L integer overflow (%f)", L);
    return -1;
  }

  double Lf = L - Li;

  msg->L.i = (s32) Li;
  msg->L.f = (u8) (Lf * MSG_OSB_LF_MULTIPLIER);

  s32 snr_fp = lround(snr * MSG_OBS_SNR_MULTIPLIER);
  if (snr < 0 || snr_fp > UINT8_MAX) {
    log_error("observation message packing: SNR integer overflow (%f)", snr);
    return -1;
  }

  msg->cn0 = (u8)snr_fp;

  msg->lock = lock_counter;

  msg->sid = prn; /* TODO prn -> sid */

  return 0;
}

void unpack_ephemeris(const msg_ephemeris_t *msg, ephemeris_t *e)
{
   e->tgd       =  msg->tgd;
   e->crs       =  msg->c_rs;
   e->crc       =  msg->c_rc;
   e->cuc       =  msg->c_uc;
   e->cus       =  msg->c_us;
   e->cic       =  msg->c_ic;
   e->cis       =  msg->c_is;
   e->dn        =  msg->dn;
   e->m0        =  msg->m0;
   e->ecc       =  msg->ecc;
   e->sqrta     =  msg->sqrta;
   e->omega0    =  msg->omega0;
   e->omegadot  =  msg->omegadot;
   e->w         =  msg->w;
   e->inc       =  msg->inc;
   e->inc_dot   =  msg->inc_dot;
   e->af0       =  msg->af0;
   e->af1       =  msg->af1;
   e->af2       =  msg->af2;
   e->toe.tow   =  msg->toe_tow;
   e->toe.wn    =  msg->toe_wn;
   e->toc.tow   =  msg->toc_tow;
   e->toc.wn    =  msg->toe_wn;
   e->valid     =  msg->valid;
   e->healthy   =  msg->healthy;
   e->prn       =  msg->sid & 0x1F; /* TODO prn -> sid */
   e->iode      =  msg->iode;
}

void pack_ephemeris(const ephemeris_t *e, msg_ephemeris_t *msg)
{
  gps_time_t toe = e->toe;
  gps_time_t toc = e->toc;
  msg->tgd       = e->tgd;
  msg->c_rs      = e->crs;
  msg->c_rc      = e->crc;
  msg->c_uc      = e->cuc;
  msg->c_us      = e->cus;
  msg->c_ic      = e->cic;
  msg->c_is      = e->cis;
  msg->dn        = e->dn;
  msg->m0        = e->m0;
  msg->ecc       = e->ecc;
  msg->sqrta     = e->sqrta;
  msg->omega0    = e->omega0;
  msg->omegadot  = e->omegadot;
  msg->w         = e->w;
  msg->inc       = e->inc;
  msg->inc_dot   = e->inc_dot;
  msg->af0       = e->af0;
  msg->af1       = e->af1;
  msg->af2       = e->af2;
  msg->toe_tow   = toe.tow;
  msg->toe_wn    = toe.wn;
  msg->toc_tow   = toc.tow;
  msg->toe_wn    = toc.wn;
  msg->valid     = e->valid;
  msg->healthy   = e->healthy;
  msg->sid       = e->prn; /* TODO: prn -> sid */
  msg->iode      = e->iode;
}

/** \} */
/** \} */

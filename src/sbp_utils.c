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

#include <math.h>

#include <libswiftnav/constants.h>
#include "sbp_utils.h"

/** \addtogroup sbp
 * \{ */

/** \defgroup sbp_utils SBP Utils
 * Convert to and from SBP message types and other useful functions.
 * \{ */

void sbp_make_gps_time(msg_gps_time_t *t_out, gps_time_t *t_in, u8 flags)
{
  t_out->wn = t_in->wn;
  t_out->tow = round(t_in->tow * 1e3);
  t_out->ns = round((t_in->tow - t_out->tow*1e-3) * 1e9);
  t_out->flags = flags;
}

void sbp_make_pos_llh(msg_pos_llh_t *pos_llh, gnss_solution *soln, u8 flags)
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

void sbp_make_pos_llh_vect(msg_pos_llh_t *pos_llh, double llh[3],
                           gps_time_t *gps_t, u8 n_used, u8 flags)
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

void sbp_make_pos_ecef(msg_pos_ecef_t *pos_ecef, gnss_solution *soln, u8 flags)
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

void sbp_make_pos_ecef_vect(msg_pos_ecef_t *pos_ecef, double ecef[3],
                            gps_time_t *gps_t, u8 n_used, u8 flags)
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

void sbp_make_vel_ned(msg_vel_ned_t *vel_ned, gnss_solution *soln, u8 flags)
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

void sbp_make_vel_ecef(msg_vel_ecef_t *vel_ecef, gnss_solution *soln, u8 flags)
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

void sbp_make_dops(msg_dops_t *dops_out, dops_t *dops_in, gps_time_t *t)
{
  dops_out->tow = round(t->tow * 1e3);
  dops_out->pdop = round(dops_in->pdop * 100);
  dops_out->gdop = round(dops_in->gdop * 100);
  dops_out->tdop = round(dops_in->tdop * 100);
  dops_out->hdop = round(dops_in->hdop * 100);
  dops_out->vdop = round(dops_in->vdop * 100);
}

void sbp_make_baseline_ecef(msg_baseline_ecef_t *baseline_ecef, gps_time_t *t,
                            u8 n_sats, double b_ecef[3], u8 flags) {
  baseline_ecef->tow = round(t->tow * 1e3);
  baseline_ecef->x = round(1e3 * b_ecef[0]);
  baseline_ecef->y = round(1e3 * b_ecef[1]);
  baseline_ecef->z = round(1e3 * b_ecef[2]);
  baseline_ecef->accuracy = 0;
  baseline_ecef->n_sats = n_sats;
  baseline_ecef->flags = flags;
}

void sbp_make_baseline_ned(msg_baseline_ned_t *baseline_ned, gps_time_t *t,
                           u8 n_sats, double b_ned[3], u8 flags) {
  baseline_ned->tow = round(t->tow * 1e3);
  baseline_ned->n = round(1e3 * b_ned[0]);
  baseline_ned->e = round(1e3 * b_ned[1]);
  baseline_ned->d = round(1e3 * b_ned[2]);
  baseline_ned->h_accuracy = 0;
  baseline_ned->v_accuracy = 0;
  baseline_ned->n_sats = n_sats;
  baseline_ned->flags = flags;
}

/** \} */
/** \} */

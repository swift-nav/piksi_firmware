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



#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>

#include "chdebug.h"

#include "error.h"
#include "sbp_piksi.h"


void unpack_obs_header(msg_obs_header_t *msg, gps_time_t* t, u8* total, u8* count)
{
  t->tow = ((double)msg->t.tow) / MSG_OBS_TOW_MULTIPLIER;
  t->wn  = msg->t.wn;
  *total = (msg->seq >> MSG_OBS_HEADER_SEQ_SHIFT);
  *count = (msg->seq & MSG_OBS_HEADER_SEQ_MASK);
}

void pack_obs_header(gps_time_t *t, u8 total, u8 count, msg_obs_header_t *msg)
{
  msg->t.tow = (u32)round(t->tow * MSG_OBS_TOW_MULTIPLIER);
  msg->t.wn  = t->wn;
  msg->seq   = ((total << MSG_OBS_HEADER_SEQ_SHIFT) |
                 (count & MSG_OBS_HEADER_SEQ_MASK));
}

void unpack_obs_content(msg_obs_content_t *msg, double *P, double *L,
                        double *snr, u16 *lock_counter, u8 *prn)
{
  *P   = ((double)msg->P) / MSG_OBS_P_MULTIPLIER;
  *L   = ((double)msg->L.Li) + (((double)msg->L.Lf) / MSG_OSB_LF_MULTIPLIER);
  *snr = ((double)msg->snr) / MSG_OBS_SNR_MULTIPLIER;
  *lock_counter = ((u16)msg->lock_counter);
  *prn = msg->prn;
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
                    msg_obs_content_t *msg)
{

  s64 P_fp = llround(P * MSG_OBS_P_MULTIPLIER);
  if (P < 0 || P_fp > UINT32_MAX) {
    printf("ERROR: observation message packing: P integer overflow (%f)\n", P);
    return -1;
  }

  msg->P = (u32)P_fp;

  double Li = floor(L);
  if (Li < INT32_MIN || Li > INT32_MAX) {
    printf("ERROR: observation message packing: L integer overflow (%f)\n", L);
    return -1;
  }

  double Lf = L - Li;

  msg->L.Li = (s32) Li;
  msg->L.Lf = (u8) (Lf * MSG_OSB_LF_MULTIPLIER);

  s32 snr_fp = lround(snr * MSG_OBS_SNR_MULTIPLIER);
  if (snr < 0 || snr_fp > UINT8_MAX) {
    printf("ERROR: observation message packing: SNR integer overflow (%f)\n",
           snr);
    return -1;
  }

  msg->snr = (u8)snr_fp;

  msg->lock_counter = lock_counter;

  msg->prn = prn;

  return 0;
}


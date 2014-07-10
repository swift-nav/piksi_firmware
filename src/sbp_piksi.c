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
                        double *snr, u8 *prn)
{
  *P   = ((double)msg->P) / MSG_OBS_P_MULTIPLIER;
  *L   = ((double)msg->L.Li) + (((double)msg->L.Lf) / MSG_OSB_LF_MULTIPLIER);
  *snr = ((double)msg->snr) / MSG_OBS_SNR_MULTIPLIER;
  *prn = msg->prn;
}

void pack_obs_content(double P, double L, double snr, u8 prn,
                      msg_obs_content_t *msg)
{

  if (P < 0 || P > INT_MAX) {
    screaming_death("observation message packing: integer overflow");
  }

  msg->P = (u32) round(P * MSG_OBS_P_MULTIPLIER);

  if (L < -(INT_MAX) || L > INT_MAX) {
    screaming_death("observation message packing: integer overflow");
  }

  double Li = floor(L);
  double Lf = L - Li;

  msg->L.Li = (s32) Li;
  msg->L.Lf = (u8) (Lf * MSG_OSB_LF_MULTIPLIER);

  if (snr < 0 || snr > ((1<<8) / MSG_OBS_SNR_MULTIPLIER)) {
    screaming_death("observation message packing: integer overflow");
  }

  msg->snr = (u8) round(snr * MSG_OBS_SNR_MULTIPLIER);

  msg->prn = prn;
}


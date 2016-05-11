/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *          Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "board.h"
#include "nap/nap_common.h"
#include "nap/track_channel.h"
#include "track.h"
#include "main.h"

#include "track.h"

#include "track_gps_l2cm.h"
#include "track_api.h"
#include "decode.h"
#include "manage.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

#include <string.h>
#include <assert.h>
#include "math.h"

#include "settings.h"
#include "signal.h"

/* Number of chips to integrate over in the short cycle interval [chips]
 * The value must be within [0..(2 * GPS_L2CM_CHIPS_NUM)].
 */
#define L2CM_TRACK_SHORT_CYCLE_INTERVAL_CHIPS 1023

/** Do L1C/A to L2 CM handover.
 *
 * The condition for the handover is the availability of bitsync on L1 C/A
 *
 * \param sample_count NAP sample count
 * \param sat L1C/A Satellite ID
 * \param nap_channel Associated NAP channel
 * \param code_phase L1CA code phase [chips]
 */
void do_l1ca_to_l2cm_handover(u32 sample_count,
                              u16 sat,
                              u8 nap_channel,
                              float code_phase)
{
  /* First, get L2C capability for the SV from NDB */
  u32 l2c_cpbl;
  // TODO: uncomment this as soon as NDB gets available
  // ndb_gps_l2cm_l2c_cap_read(&l2c_cpbl);

  // TODO: remove this as soon as NDB gets available
  // GPS PRNs with L2C capability:
  // 01 03 05 06 07 08 09 10 12 15 17 24 25 26 27 29 30 31 32
  // as per http://tinyurl.com/zj5q62h
  l2c_cpbl = (u32)1 << (1 - 1);
  l2c_cpbl |= (u32)1 << (3 - 1);
  l2c_cpbl |= (u32)1 << (5 - 1);
  l2c_cpbl |= (u32)1 << (6 - 1);
  l2c_cpbl |= (u32)1 << (7 - 1);
  l2c_cpbl |= (u32)1 << (8 - 1);
  l2c_cpbl |= (u32)1 << (9 - 1);
  l2c_cpbl |= (u32)1 << (10 - 1);
  l2c_cpbl |= (u32)1 << (12 - 1);
  l2c_cpbl |= (u32)1 << (15 - 1);
  l2c_cpbl |= (u32)1 << (17 - 1);
  l2c_cpbl |= (u32)1 << (24 - 1);
  l2c_cpbl |= (u32)1 << (25 - 1);
  l2c_cpbl |= (u32)1 << (26 - 1);
  l2c_cpbl |= (u32)1 << (27 - 1);
  l2c_cpbl |= (u32)1 << (29 - 1);
  l2c_cpbl |= (u32)1 << (30 - 1);
  l2c_cpbl |= (u32)1 << (31 - 1);
  l2c_cpbl |= (u32)1 << (32 - 1);

  /* compose SID: same SV, but code is L2 CM */
  gnss_signal_t sid = {
    .sat  = sat,
    .code = CODE_GPS_L2CM
  };

  if (0 == (l2c_cpbl & ((u32)1 << (sat - 1)))) {
    log_info_sid(sid, "SV does not support L2C signal");
    return;
  }

  if ((code_phase < 0) ||
      ((code_phase > 0.5) && (code_phase < (GPS_L1CA_CHIPS_NUM - 0.5)))) {
    log_warn_sid(sid, "Unexpected L1C/A to L2C handover code phase: %f",
                 code_phase);
    return;
  }

  if (code_phase > (GPS_L1CA_CHIPS_NUM - 0.5)) {
    code_phase = 2 * GPS_L2CM_CHIPS_NUM - (GPS_L1CA_CHIPS_NUM - code_phase);
  }

  /* recalculate doppler freq for L2 from L1*/
  double carrier_freq = tracking_channel_carrier_freq_get(nap_channel) *
                        GPS_L2_HZ / GPS_L1_HZ;

  /* get initial cn0 from parent L1 channel */
  float cn0_init = tracking_channel_cn0_get(nap_channel);

  s8 elevation = tracking_channel_evelation_degrees_get(nap_channel);

  tracking_startup_params_t startup_params = {
    .sid                = sid,
    .sample_count       = sample_count,
    .carrier_freq       = carrier_freq,
    .code_phase         = code_phase,
    .chips_to_correlate = L2CM_TRACK_SHORT_CYCLE_INTERVAL_CHIPS,
    .cn0_init           = cn0_init,
    .elevation          = elevation
  };

  if (tracking_startup_request(&startup_params)) {
    log_info_sid(sid, "L2 CM handover done");
  } else {
    log_error_sid(sid, "Failed to start L2C tracking");
  }
}

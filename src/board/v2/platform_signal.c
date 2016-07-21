/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "platform_signal.h"

#include "track/track_gps_l1ca.h"

#include "decode/decode_gps_l1ca.h"

#include "ndb.h"

void platform_track_setup(void)
{
  track_gps_l1ca_register();
}

void platform_decode_setup(void)
{
  decode_gps_l1ca_register();
}

void platform_ndb_init(void)
{
  ndb_ephemeris_init();
  ndb_almanac_init();
}

void platform_ndb_sbp_updates(void)
{
  ndb_ephemeris_sbp_update();
  ndb_almanac_sbp_update();
}


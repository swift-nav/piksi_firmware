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

#ifndef SWIFTNAV_PLATFORM_SIGNAL_H
#define SWIFTNAV_PLATFORM_SIGNAL_H

/* Platform-specific code support */
#define CODE_GPS_L1CA_SUPPORT     1
#define CODE_GPS_L2CM_SUPPORT     0
#define CODE_SBAS_L1CA_SUPPORT    1
#define CODE_GLO_L1CA_SUPPORT     0
#define CODE_GLO_L2CA_SUPPORT     0

/* Tracker configuration */
#define NUM_TRACKER_CHANNELS      12
#define NUM_GPS_L1CA_TRACKERS     12

/* Decoder configuration */
#define NUM_DECODER_CHANNELS      12
#define NUM_GPS_L1CA_DECODERS     12

void platform_track_setup(void);
void platform_decode_setup(void);

void platform_ndb_init(void);
void platform_ndb_sbp_updates(void);

#endif /* SWIFTNAV_PLATFORM_SIGNAL_H */

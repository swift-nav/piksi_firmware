/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 * Contact: Adel Mamin <adelm@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_TRACK_GPS_L2CM_H
#define SWIFTNAV_TRACK_GPS_L2CM_H

#include <libswiftnav/common.h>

void do_l1ca_to_l2cm_handover(u32 sample_count,
                              u16 sat,
                              u8 nap_channel,
                              float code_phase);

#endif /* SWIFTNAV_TRACK_GPS_L2CM_H */

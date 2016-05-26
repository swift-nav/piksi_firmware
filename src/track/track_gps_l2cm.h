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

#ifdef TRACK_GPS_L2CM_INTERNAL
# define L2C_WEAK
#else
# define L2C_WEAK __attribute__ ((weak, alias ("l2c_not_implemented")))
#endif  /* TRACK_GPS_L2CM_INTERNAL */

int l2c_not_implemented() __attribute__ ((weak));
inline int l2c_not_implemented() { return -1; }

/* not weak as it is used in L2C builds only */
void track_gps_l2cm_register(void);

void do_l1ca_to_l2cm_handover(u32 sample_count,
                              u16 sat,
                              float code_phase,
                              double carrier_freq,
                              float cn0_init) L2C_WEAK;

#endif /* SWIFTNAV_TRACK_GPS_L2CM_H */

/*
 * Copyright (C) 2013 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_NMEA_H
#define SWIFTNAV_NMEA_H

#include <libswiftnav/common.h>
#include <libswiftnav/pvt.h>

#include "track.h"

void nmea_gpgga(gnss_solution *soln, dops_t *dops);
void nmea_gpgsa(tracking_channel_t *chans, dops_t *dops);
void nmea_gpgsv(u8 n_used, navigation_measurement_t *nav_meas,
                gnss_solution *soln);

#endif  /* SWIFTNAV_NMEA_H */


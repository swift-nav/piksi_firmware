/*
 * Copyright (C) 2012-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SIMULATOR_DATA_H
#define SWIFTNAV_SIMULATOR_DATA_H

#include <libswiftnav/common.h>
#include <libswiftnav/almanac.h>

extern u16 simulation_week_number;
extern u8 simulation_num_almanacs;
extern double simulation_sats_pos[][3];
extern double simulation_sats_vel[][3];
extern u32 simulation_fake_carrier_bias[];
extern const almanac_t simulation_almanacs[];

#endif /* SWIFTNAV_SIMULATOR_DATA_H */

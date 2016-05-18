/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_DECODE_GPS_L2C_H
#define SWIFTNAV_DECODE_GPS_L2C_H

#include <libswiftnav/common.h>

#undef L2C_WEAK

#ifdef FEATURE_L2C
# define L2C_WEAK
#else
# define L2C_WEAK __attribute__ ((weak, alias ("l2c_not_implemented")))
#endif  /* FEATURE_L2C */

int l2c_decode_not_implemented() __attribute__ ((weak));
inline int l2c_decode_not_implemented() { return -1; }

void decode_gps_l2c_register(void) L2C_WEAK;

#endif  /* SWIFTNAV_DECODE_GPS_L2C_H */

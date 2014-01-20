/*
 * Copyright (C) 2013-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_RTCM_H
#define SWIFTNAV_RTCM_H

#include <libswiftnav/common.h>
#include <libswiftnav/gpstime.h>
#include <libswiftnav/track.h>
#include <libswiftnav/ephemeris.h>

typedef struct {
  u32 nbyte;          /* number of bytes in message buffer */
  u32 nbit;           /* number of bits in word buffer */
  u32 len;            /* message length (bytes) */
  u8 buff[1200]; /* message buffer */
  gps_time_t time;
  u8 n;
  u8 prn;
  navigation_measurement_t obs[64];
  ephemeris_t *eph;
} rtcm_t;

int gen_rtcm3(rtcm_t *rtcm, int type, int sync);

#endif



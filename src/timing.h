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

#ifndef SWIFTNAV_TIME_H
#define SWIFTNAV_TIME_H

#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/rcc.h>

#include <libswiftnav/common.h>
#include <libswiftnav/gpstime.h>

/** \addtogroup timing Timing
 * \{ */

typedef enum {
  TIME_UNKNOWN = 0,
  TIME_GUESS,
  TIME_COARSE,
  TIME_FINE
} time_quality_t;

extern time_quality_t time_quality;

#define TICK_FREQ (rcc_ppre1_frequency/16)

/** \} */

void time_setup();
gps_time_t get_current_time();
void set_time(time_quality_t quality, gps_time_t t);
void set_time_fine(u64 tc, gps_time_t t);
gps_time_t rx2gpstime(double tc);
double gps2rxtime(gps_time_t t);

u32 time_ticks();

/** \addtogroup timing Timing
 * \{ */

#define DO_EVERY_COUNTS(n, cmd) do { \
  static u32 last_count = 0; \
  if (time_ticks() - last_count >= n) { \
    last_count = time_ticks(); \
    cmd; \
  } \
} while(0)

/** \} */

#endif


/*
 * Copyright (C) 2013 Fergus Noble <fergusnoble@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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


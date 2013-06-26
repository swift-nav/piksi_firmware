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

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>

#include <libswiftnav/common.h>
#include <libswiftnav/gpstime.h>

/** \addtogroup timing Timing
 * \{ */

typedef enum {
  TIME_UNKNOWN = 0, /**< GPS time is completely unknown, estimate invalid. */
  TIME_GUESS,       /**< GPS time is just a guess, it could be off by weeks or
                         just totally incorrect. */
  TIME_COARSE,      /**< GPS time is known roughly, within 1 second. */
  TIME_FINE         /**< GPS time is known precisely with reference to the
                         local SwiftNAP timer. */
} time_quality_t;

/** \} */

extern time_quality_t time_quality;

#define RX_DT_NOMINAL (1.0 / SAMPLE_FREQ)
#define TICK_FREQ     (rcc_ppre1_frequency / 16)

void timing_setup(void);
gps_time_t get_current_time(void);
void set_time(time_quality_t quality, gps_time_t t);
void set_time_fine(u64 tc, gps_time_t t);
gps_time_t rx2gpstime(double tc);
double gps2rxtime(gps_time_t t);

void tick_timer_setup(void);
u32 time_ticks(void);

/** \addtogroup timing Timing
 * \{ */

/** Perform an operation at most every n time ticks.
 * Used for rough scheduling. Every time this macro is called, it compares the
 * current tick count to the count on the previous call to the macro. If the
 * difference is greater than or equal to n then cmd is executed.
 *
 * \note This macro can be used independantly in multiple places but cannot be
 *       nested.
 *
 * \param n   Minimum number of ticks between successive executions of cmd.
 * \param cmd Code block to execute.
 */
#define DO_EVERY_TICKS(n, cmd) do {       \
    static u32 last_count = 0;            \
    if (time_ticks() - last_count >= n) { \
      last_count = time_ticks();          \
      cmd;                                \
    }                                     \
} while (0)

/** \} */

#endif


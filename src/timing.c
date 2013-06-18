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

#include <math.h>
#include <time.h>
#include <stdio.h>

#include "main.h"
#include "timing.h"
#include "sbp.h"
#include "board/nap/nap_common.h"

/** \defgroup timing Timing
 * Convert between receiver time and GPS time.
 * \{ */

time_quality_t time_quality = TIME_UNKNOWN;

static gps_time_t rx_t0;
#define RX_DT_NOMINAL (1.0 / SAMPLE_FREQ)
static double rx_dt = RX_DT_NOMINAL;

/** Get current GPS time (referenced to initial GPS time estimate).
 * \return Current GPS time.
 */
gps_time_t get_current_time()
{
  /* TODO: Return invalid when TIME_UNKNOWN. */
  /* TODO: Think about what happens when nap_timing_count overflows. */
  u64 tc = nap_timing_count();
  gps_time_t t = rx2gpstime(tc);
  return t;
}

/** Set initial GPS time estimate.
 * \param quality Quality of the time estimate.
 * \param t GPS time estimate.
 */
void set_time(time_quality_t quality, gps_time_t t)
{
  set_time_fine(nap_timing_count(), t);
  time_quality = quality;
  time_t unix_t = gps2time(t);
  printf("Time set to: %s (quality=%d)\n", ctime(&unix_t), quality);
}

/** Set initial GPS time estimate - quality fine.
 * \param tc Timing count in units of RX_DT_NOMINAL.
 * \param t GPS time estimate associated with timing count.
 */
void set_time_fine(u64 tc, gps_time_t t)
{
  /* TODO: proper first order estimator here? */
  rx_dt = RX_DT_NOMINAL;
  rx_t0 = t;
  rx_t0.tow -= tc * rx_dt;
  rx_t0 = normalize_gps_time(rx_t0);
  time_quality = TIME_FINE;
}

/** Convert receiver time to GPS time (referenced to initial GPS time estimate).
 * \param tc Timing count in units of RX_DT_NOMINAL.
 * \return GPS time corresponding to Timing count.
 */
gps_time_t rx2gpstime(double tc)
{
  gps_time_t t = rx_t0;
  t.tow += tc * rx_dt;
  t = normalize_gps_time(t);
  return t;
}

/** Convert GPS time (relative to initial GPS time estimate) to receiver time.
 * \param t gps_time_t to convert.
 * \return Timing count in units of RX_DT_NOMINAL.
 */
double gps2rxtime(gps_time_t t)
{
  return gpsdifftime(t, rx_t0) / rx_dt;
}

/** Callback to set receiver GPS time estimate. */
void set_time_callback(u8 buff[])
{
  gps_time_t* t = (gps_time_t*)buff;
  set_time(TIME_COARSE, *t);
}

/** Setup timer to use as a rough system timing count. */
void time_setup()
{
  /* TODO: Perhaps setup something to check for nap_timing_count overflows
   * periodically. */
  static msg_callbacks_node_t set_time_node;
  sbp_register_callback(MSG_SET_TIME, &set_time_callback, &set_time_node);

  /* Setup Timer 2 as our global tick counter. */
  RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;

  /* Set timer prescale to divide APB bus to the tick frequency.
   *
   * NOTE: This will only work for ppre1_freq that is an integer
   *       multiple of the tick frequency.
   * NOTE: Assumes APB1 prescale != 1, see Ref Man pg. 84
   */
  timer_set_prescaler(TIM2, 2*rcc_ppre1_frequency/TICK_FREQ - 1);

  /* Set time auto-reload value to the longest possible period. */
  timer_set_period(TIM2, 0xFFFFFFFF);

  /* Enable timer */
  TIM2_CNT = 0;
  timer_generate_event(TIM2, TIM_EGR_UG);
  timer_enable_counter(TIM2);
}

/** Return current rough system timing count.
 * Used for rough timing - mainly to schedule housekeeping operations that
 * do not need GPS time precision.
 * \return Rough system timing count.
 */
u32 time_ticks() {
  /* TODO: think about overflows. */

  return TIM2_CNT;
}

/** \} */

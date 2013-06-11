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

#include <math.h>
#include <time.h>
#include <stdio.h>

#include "main.h"
#include "timing.h"
#include "sbp.h"
#include "board/nap/nap_common.h"

time_quality_t time_quality = TIME_UNKNOWN;

static gps_time_t rx_t0;
#define RX_DT_NOMINAL (1.0 / SAMPLE_FREQ)
static double rx_dt = RX_DT_NOMINAL;

gps_time_t get_current_time()
{
  /* TODO: Return invalid when TIME_UNKNOWN. */
  /* TODO: Think about what happens when nap_timing_count overflows. */
  u64 tc = nap_timing_count();
  gps_time_t t = rx2gpstime(tc);
  return t;
}

void set_time_coarse(gps_time_t t)
{
  set_time_fine(nap_timing_count(), t);
  time_quality = TIME_COARSE;
  time_t unix_t = gps2time(t);
  printf("Time set to: %s (coarse)\n", ctime(&unix_t));
}

void set_time_fine(u64 tc, gps_time_t t)
{
  /* TODO: proper first order estimator here? */
  rx_dt = RX_DT_NOMINAL;
  rx_t0 = t;
  rx_t0.tow -= tc * rx_dt;
  rx_t0 = normalize_gps_time(rx_t0);
  time_quality = TIME_FINE;
}

gps_time_t rx2gpstime(double tc)
{
  gps_time_t t = rx_t0;
  t.tow += tc * rx_dt;
  t = normalize_gps_time(t);
  return t;
}

double gps2rxtime(gps_time_t t)
{
  return gpsdifftime(t, rx_t0) / rx_dt;
}

void set_time_callback(u8 buff[])
{
  gps_time_t* t = (gps_time_t*)buff;
  set_time_coarse(*t);
}

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

u32 time_ticks() {
  /* TODO: think about overflows. */

  return TIM2_CNT;
}


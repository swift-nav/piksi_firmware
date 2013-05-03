/*
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
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

#ifndef SWIFTNAV_TRACK_H
#define SWIFTNAV_TRACK_H

#include <libopencm3/cm3/common.h>
#include <libswiftnav/nav_msg.h>
#include <libswiftnav/track.h>

#include "swift_nap_io.h"

#define I_FILTER_COEFF 4
#define Q_FILTER_COEFF 10

typedef enum {
  TRACKING_DISABLED = 0,
  TRACKING_RUNNING
} tracking_state_t;

typedef struct {
  tracking_state_t state; 
  u32 update_count;
  u32 TOW_ms;
  u32 snr_threshold_count;
  u8 prn;

  u32 sample_count;
  u32 code_phase_early;

  double code_phase_rate;
  u32 code_phase_rate_fp;
  u32 code_phase_rate_fp_prev;
  double carrier_freq;

  /* Loop filter state. */
  double dll_igain, dll_pgain;
  double pll_igain, pll_pgain;
  double dll_disc;
  double pll_disc;

  /* SNR filter state. */
  u32 I_filter, Q_filter;
  float snr;

  u16 corr_sample_count;
  corr_t cs[3];

  nav_msg_t nav_msg;
} tracking_channel_t;

extern tracking_channel_t tracking_channel[MAX_TRACK_N_CHANNELS];

float propagate_code_phase(float code_phase, float carrier_freq, u32 n_samples);
void tracking_channel_init(u8 channel, u8 prn, float carrier_freq, u32 start_sample_count);

void tracking_channel_get_corrs(u8 channel);
void tracking_channel_update(u8 channel);
void tracking_channel_disable(u8 channel);
void tracking_update_measurement(u8 channel, channel_measurement_t *meas);
float tracking_channel_snr(u8 channel);

#endif

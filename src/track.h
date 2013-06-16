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

#include "board/nap/nap_common.h"
#include "board/nap/track_channel.h"

/** \addtogroup tracking
 * \{ */

#define I_FILTER_COEFF 4
#define Q_FILTER_COEFF 10

/** Tracking channel states. */
typedef enum {
  TRACKING_DISABLED = 0,
  TRACKING_RUNNING = 1
} tracking_state_t;

/** Message struct for SBP tracking state message. */
typedef struct __attribute__((packed)) {
  tracking_state_t state; /**< State of the tracking channel. */
  u8 prn;                 /**< PRN being tracked by the tracking channel. */
  float cn0;              /**< SNR of the tracking channel. */
} tracking_state_msg_t;

/** Tracking channel parameters as of end of last correlation period. */
typedef struct {
  tracking_state_t state;      /**< Tracking channel state. */
  /* TODO : u32's big enough? */
  u32 update_count;            /**< Total number of tracking channel ms updates. */
  s32 TOW_ms;                  /**< TOW in ms. */
  u32 snr_threshold_count;     /**< Number of ms tracking channel's SNR has been above a certain margin. */
  u8 prn;                      /**< CA Code (0-31) channel is tracking. */
  u32 sample_count;            /**< Total num samples channel has tracked for. */
  u32 code_phase_early;        /**< Early code phase. */
  comp_tl_state_t tl_state;    /**< Tracking loop filter state. */
  double code_phase_rate;      /**< Code phase rate in chips/s. */
  u32 code_phase_rate_fp;      /**< Code phase rate in NAP register units. */
  u32 code_phase_rate_fp_prev; /**< Previous code phase rate in NAP register units. */
  double carrier_freq;         /**< Carrier frequency in chips/s. */
  u32 I_filter;                /**< Filtered Prompt I correlations. */
  u32 Q_filter;                /**< Filtered Prompt Q correlations. */
  u16 corr_sample_count;       /**< Number of samples in correlation period. */
  corr_t cs[3];                /**< EPL correlation results in correlation period. */
  nav_msg_t nav_msg;           /**< Navigation message of channel SV. */
} tracking_channel_t;

/** \} */

/* Assuming we will never have a greater number of tracking channels than 12
 * We have to declare the number here as the number of tracking channels in
 * the FPGA is read at runtime. */
extern tracking_channel_t tracking_channel[NAP_MAX_N_TRACK_CHANNELS];

float propagate_code_phase(float code_phase, float carrier_freq, u32 n_samples);
void tracking_channel_init(u8 channel, u8 prn, float carrier_freq, u32 start_sample_count);

void tracking_channel_get_corrs(u8 channel);
void tracking_channel_update(u8 channel);
void tracking_channel_disable(u8 channel);
void tracking_update_measurement(u8 channel, channel_measurement_t *meas);
float tracking_channel_snr(u8 channel);
void tracking_send_state();

#endif

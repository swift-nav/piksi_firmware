/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_TRACK_H
#define SWIFTNAV_TRACK_H

#include <libsbp/tracking.h>
#include <libswiftnav/common.h>
#include <libswiftnav/nav_msg.h>
#include <libswiftnav/track.h>

#include "board/nap/nap_common.h"
#include "board/nap/track_channel.h"

/** \addtogroup tracking
 * \{ */

#define I_FILTER_COEFF 4
#define Q_FILTER_COEFF 10

#define TRACKING_DISABLED 0 /**< Tracking channel disabled state. */
#define TRACKING_RUNNING  1 /**< Tracking channel running state. */

extern u8 n_rollovers;

/** Tracking channel parameters as of end of last correlation period. */
typedef struct {
  u8 state;                    /**< Tracking channel state. */
  /* TODO : u32's big enough? */
  u32 update_count;            /**< Total number of tracking channel ms updates. */
  s32 TOW_ms;                  /**< TOW in ms. */
  u32 snr_above_threshold_count;     /**< update_count value when SNR was last above a certain margin. */
  u32 snr_below_threshold_count;     /**< update_count value when SNR was last below a certain margin. */
  u8 prn;                      /**< CA Code (0-31) channel is tracking. */
  u32 sample_count;            /**< Total num samples channel has tracked for. */
  u32 code_phase_early;        /**< Early code phase. */
  aided_tl_state_t tl_state;   /**< Tracking loop filter state. */
  double code_phase_rate;      /**< Code phase rate in chips/s. */
  u32 code_phase_rate_fp;      /**< Code phase rate in NAP register units. */
  u32 code_phase_rate_fp_prev; /**< Previous code phase rate in NAP register units. */
  s64 carrier_phase;           /**< Carrier phase in NAP register units. */
  s32 carrier_freq_fp;         /**< Carrier frequency in NAP register units. */
  s32 carrier_freq_fp_prev;    /**< Previous carrier frequency in NAP register units. */
  double carrier_freq;         /**< Carrier frequency Hz. */
  u32 I_filter;                /**< Filtered Prompt I correlations. */
  u32 Q_filter;                /**< Filtered Prompt Q correlations. */
  u32 corr_sample_count;       /**< Number of samples in correlation period. */
  corr_t cs[3];                /**< EPL correlation results in correlation period. */
  nav_msg_t nav_msg;           /**< Navigation message of channel SV. */
  u16 lock_counter;            /**< Lock counter. Increments when tracking new signal. */
} tracking_channel_t;

/** \} */

/* Assuming we will never have a greater number of tracking channels than 12
 * We have to declare the number here as the number of tracking channels in
 * the FPGA is read at runtime. */
/* TODO: NAP_MAX_N_TRACK_CHANNELS is a duplicate of MAX_CHANNELS */
extern tracking_channel_t tracking_channel[NAP_MAX_N_TRACK_CHANNELS];

void initialize_lock_counters(void);

float propagate_code_phase(float code_phase, float carrier_freq, u32 n_samples);
void tracking_channel_init(u8 channel, u8 prn, float carrier_freq,
                           u32 start_sample_count, float snr);

void tracking_channel_get_corrs(u8 channel);
void tracking_channel_update(u8 channel);
void tracking_channel_disable(u8 channel);
void tracking_update_measurement(u8 channel, channel_measurement_t *meas);
float tracking_channel_snr(u8 channel);
void tracking_send_state(void);

#endif

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
#include <libswiftnav/signal.h>
#include <libswiftnav/bit_sync.h>

#include "board/nap/nap_common.h"
#include "board/nap/track_channel.h"

/** \addtogroup tracking
 * \{ */

#define TRACKING_DISABLED 0 /**< Tracking channel disabled state. */
#define TRACKING_RUNNING  1 /**< Tracking channel running state. */
#define TRACKING_ELEVATION_UNKNOWN 100 /* Ensure it will be above elev. mask */
extern u8 n_rollovers;

#define NAV_BIT_FIFO_SIZE 32 /**< Size of nav bit FIFO. Must be a power of 2 */

typedef struct {
  s8 soft_bit;
} nav_bit_fifo_element_t;

typedef u8 nav_bit_fifo_index_t;

typedef struct {
  nav_bit_fifo_index_t read_index;
  nav_bit_fifo_index_t write_index;
  nav_bit_fifo_element_t elements[NAV_BIT_FIFO_SIZE];
} nav_bit_fifo_t;

typedef struct {
  s32 TOW_ms;
  s8 bit_polarity;
  nav_bit_fifo_index_t read_index;
  bool valid;
} nav_time_sync_t;

/** Tracking channel parameters as of end of last correlation period. */
typedef struct {
  u8 state;                    /**< Tracking channel state. */
  /* TODO : u32's big enough? */
  u32 update_count;            /**< Number of ms channel has been running */
  u32 mode_change_count;       /**< update_count at last mode change. */
  u32 cn0_above_drop_thres_count;
                               /**< update_count value when SNR was
                                  last above a certain margin. */
  u32 ld_opti_locked_count;    /**< update_count value when optimistic
                                  phase detector last "locked". */
  s32 TOW_ms;                  /**< TOW in ms. */
  u32 nav_bit_TOW_offset_ms;   /**< Time since last nav bit was appended to the nav bit FIFO */
  u32 cn0_below_threshold_count;     /**< update_count value when SNR was last below a certain margin. */
  gnss_signal_t sid;           /**< Satellite signal being tracked. */
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
  u32 corr_sample_count;       /**< Number of samples in correlation period. */
  corr_t cs[3];                /**< EPL correlation results in correlation period. */
  bit_sync_t bit_sync;         /**< Bit sync state. */
  s8 bit_polarity;             /**< Polarity of nav message bits. */
  u16 lock_counter;            /**< Lock counter. Increments when tracking new signal. */
  cn0_est_state_t cn0_est;     /**< C/N0 Estimator. */
  float cn0;                   /**< Current estimate of C/N0. */
  u8 int_ms;                   /**< Integration length. */
  u8 next_int_ms;              /**< Integration length for the next cycle. */
  bool short_cycle;            /**< Set to true when a short 1ms integration is requested. */
  bool output_iq;              /**< Set if this channel should output I/Q samples on SBP. */
  u8 stage;                    /**< 0 = First-stage. 1 ms integration.
                                    1 = Second-stage. After nav bit sync,
                                    retune loop filters and typically (but
                                    not necessarily) use longer integration. */
  s8 elevation;                /**< Elevation angle, degrees */
  alias_detect_t alias_detect; /**< Alias lock detector. */
  lock_detect_t lock_detect;   /**< Phase-lock detector state. */
  nav_bit_fifo_t nav_bit_fifo; /**< FIFO for navigation message bits. */
  nav_time_sync_t nav_time_sync;  /**< Used to sync time decoded from navigation
                                       message back to tracking channel. */
} tracking_channel_t;

/** \} */

/* Assuming we will never have a greater number of tracking channels than 12
 * We have to declare the number here as the number of tracking channels in
 * the FPGA is read at runtime. */
/* TODO: NAP_MAX_N_TRACK_CHANNELS is a duplicate of MAX_CHANNELS */
extern tracking_channel_t tracking_channel[NAP_MAX_N_TRACK_CHANNELS];

void initialize_lock_counters(void);

float propagate_code_phase(float code_phase, float carrier_freq, u32 n_samples);
s8 nav_bit_quantize(s32 bit_integrate);

void tracking_channel_init(u8 channel, gnss_signal_t sid, float carrier_freq,
                           u32 start_sample_count, float cn0_init, s8 elevation);

void tracking_channel_update(u8 channel);
void tracking_channel_disable(u8 channel);
void tracking_channel_ambiguity_unknown(u8 channel);
void tracking_update_measurement(u8 channel, channel_measurement_t *meas);
void tracking_send_state(void);
void tracking_setup(void);
void tracking_drop_satellite(gnss_signal_t sid);
bool tracking_channel_nav_bit_get(u8 channel, s8 *soft_bit);
bool tracking_channel_time_sync(u8 channel, s32 TOW_ms, s8 bit_polarity);

#endif

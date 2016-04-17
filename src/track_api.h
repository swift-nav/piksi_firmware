/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_TRACK_API_H
#define SWIFTNAV_TRACK_API_H

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>

#include "board/nap/nap_common.h"
#include "board/nap/track_channel.h"

/** \addtogroup track_api
 * \{ */

typedef u32 update_count_t;

typedef struct {
  update_count_t update_count; /**< Number of ms channel has been running */
  update_count_t mode_change_count;
                               /**< update_count at last mode change. */
  update_count_t cn0_below_use_thres_count;
                               /**< update_count value when SNR was
                                    last below the use threshold. */
  update_count_t cn0_above_drop_thres_count;
                               /**< update_count value when SNR was
                                    last above the drop threshold. */
  update_count_t ld_opti_locked_count;
                               /**< update_count value when optimistic
                                    phase detector last "locked". */
  update_count_t ld_pess_unlocked_count;
                               /**< update_count value when pessimistic
                                    phase detector last "unlocked". */
  s32 TOW_ms;                  /**< TOW in ms. */
  u32 sample_count;            /**< Total num samples channel has tracked for. */
  double code_phase_early;        /**< Early code phase. */
  double code_phase_rate;      /**< Code phase rate in chips/s. */
  double carrier_phase;           /**< Carrier phase in NAP register units. */
  double carrier_freq;         /**< Carrier frequency Hz. */
  float cn0;                   /**< Current estimate of C/N0. */
} tracker_common_data_t;

typedef void tracker_data_t;
typedef void tracker_context_t;

/** Instance of a tracker implementation. */
typedef struct {
  /** true if tracker is in use. */
  bool active;
  /** Pointer to implementation-specific data used by tracker instance. */
  tracker_data_t *data;
} tracker_t;

/** Info associated with a tracker channel. */
typedef struct {
  gnss_signal_t sid;            /**< Current signal being decoded. */
  u8 nap_channel;               /**< Associated NAP channel. */
  tracker_context_t *context;   /**< Current context for library functions. */
} tracker_channel_info_t;

/** Tracker interface function template. */
typedef void (*tracker_interface_function_t)(
                 const tracker_channel_info_t *channel_info,
                 tracker_common_data_t *common_data,
                 tracker_data_t *tracker_data);

/** Interface to a tracker implementation. */
typedef struct {
  /** Code type for which the implementation may be used. */
  enum code code;
  /** Init function. Called to set up tracker instance when tracking begins. */
  tracker_interface_function_t init;
  /** Disable function. Called when tracking stops. */
  tracker_interface_function_t disable;
  /** Update function. Called when new correlation outputs are available. */
  tracker_interface_function_t update;
  /** Array of tracker instances used by this interface. */
  tracker_t *trackers;
  /** Number of tracker instances in trackers array. */
  u8 num_trackers;
} tracker_interface_t;

/** List element passed to tracker_interface_register(). */
typedef struct tracker_interface_list_element_t {
  const tracker_interface_t *interface;
  struct tracker_interface_list_element_t *next;
} tracker_interface_list_element_t;

/** \} */

void tracker_interface_register(tracker_interface_list_element_t *element);

/* Tracker instance API functions. Must be called from within an
 * interface function. */
void tracker_correlations_read(tracker_context_t *context, corr_t *cs,
                               u32 *sample_count,
                               double *code_phase, double *carrier_phase);
void tracker_retune(tracker_context_t *context, s32 carrier_freq_fp,
                    u32 code_phase_rate_fp, u8 rollover_count);
s32 tracker_tow_update(tracker_context_t *context, s32 current_TOW_ms,
                       u32 int_ms);
void tracker_bit_sync_update(tracker_context_t *context, u32 int_ms,
                             s32 corr_prompt_real);
u8 tracker_bit_length_get(tracker_context_t *context);
bool tracker_bit_aligned(tracker_context_t *context);
void tracker_ambiguity_unknown(tracker_context_t *context);
void tracker_correlations_send(tracker_context_t *context, const corr_t *cs);

#endif

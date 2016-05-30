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

#include "track_api.h"
#include "track_internal.h"
#include "track.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>

#include <ch.h>
#include <assert.h>

#include "sbp.h"
#include "sbp_utils.h"
#include "signal.h"

/** \defgroup track_api Tracking API
 * API functions used by tracking channel implementations.
 * \{ */

#define GPS_WEEK_LENGTH_ms (1000 * WEEK_SECS)

/** Register a tracker interface to enable tracking for a code type.
 *
 * \note element and all subordinate data must be statically allocated!
 *
 * \param element   Struct describing the interface to register.
 */
void tracker_interface_register(tracker_interface_list_element_t *element)
{
  /* p_next = address of next pointer which must be updated */
  tracker_interface_list_element_t **p_next = tracker_interface_list_ptr_get();

  while (*p_next != 0)
    p_next = &(*p_next)->next;

  element->next = 0;
  *p_next = element;
}

/** Read correlations from the NAP for a tracker channel.
 *
 * \param context         Tracker context.
 * \param cs              Output array of correlations.
 * \param sample_count    Output sample count.
 */
void tracker_correlations_read(tracker_context_t *context, corr_t *cs,
                               u32 *sample_count,
                               double *code_phase, double *carrier_phase)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  /* Read NAP CORR register */
  nap_track_read_results(channel_info->nap_channel, sample_count, cs,
                         code_phase, carrier_phase);
}

/** Write the NAP update register for a tracker channel.
 *
 * \param context             Tracker context.
 * \param carrier_freq_fp     Carrier frequency in NAP register units.
 * \param code_phase_rate_fp  Code phase rate in NAP register units.
 * \param chips_to_correlate  Number of code chips to integrate over.
 */
void tracker_retune(tracker_context_t *context, double carrier_freq,
                    double code_phase_rate, u32 chips_to_correlate)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  /* Write NAP UPDATE register. */
  nap_track_update(channel_info->nap_channel,
                   carrier_freq, code_phase_rate, chips_to_correlate, 0);
}

/** Update the TOW for a tracker channel.
 *
 * \param context           Tracker context.
 * \param current_TOW_ms    Current TOW (ms).
 * \param int_ms            Integration period (ms).
 *
 * \return Updated TOW (ms).
 */
s32 tracker_tow_update(tracker_context_t *context, s32 current_TOW_ms,
                       u32 int_ms)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  /* Latch TOW from nav message if pending */
  s32 pending_TOW_ms;
  s8 pending_bit_polarity;
  nav_bit_fifo_index_t pending_TOW_read_index;
  if (nav_time_sync_get(&internal_data->nav_time_sync, &pending_TOW_ms,
                        &pending_bit_polarity, &pending_TOW_read_index)) {

    /* Compute time since the pending data was read from the FIFO */
    nav_bit_fifo_index_t fifo_length =
      NAV_BIT_FIFO_INDEX_DIFF(internal_data->nav_bit_fifo.write_index,
                              pending_TOW_read_index);
    u32 fifo_time_diff_ms = fifo_length * internal_data->bit_sync.bit_length;

    /* Add full bit times + fractional bit time to the specified TOW */
    s32 TOW_ms = pending_TOW_ms + fifo_time_diff_ms +
                   internal_data->nav_bit_TOW_offset_ms;

    if (TOW_ms >= GPS_WEEK_LENGTH_ms)
      TOW_ms -= GPS_WEEK_LENGTH_ms;

    /* Warn if updated TOW does not match the current value */
    if ((current_TOW_ms != TOW_INVALID) && (current_TOW_ms != TOW_ms)) {
      log_warn_sid(channel_info->sid, "TOW mismatch: %ld, %lu", current_TOW_ms, TOW_ms);
    }
    current_TOW_ms = TOW_ms;
    internal_data->bit_polarity = pending_bit_polarity;
  }

  internal_data->nav_bit_TOW_offset_ms += int_ms;

  if (current_TOW_ms != TOW_INVALID) {
    /* Have a valid time of week - increment it. */
    current_TOW_ms += int_ms;
    if (current_TOW_ms >= GPS_WEEK_LENGTH_ms)
      current_TOW_ms -= GPS_WEEK_LENGTH_ms;
    /* TODO: maybe keep track of week number in channel state, or
       derive it from system time */
  }

  return current_TOW_ms;
}

/** Set bit sync phase reference
 *
 * \param context           Tracker context.
 * \param bit_phase_ref     Bit phase reference.
 */
void tracker_bit_sync_set(tracker_context_t *context, s8 bit_phase_ref)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  bit_sync_set(&internal_data->bit_sync, bit_phase_ref);
}

/** Update bit sync and output navigation message bits for a tracker channel.
 *
 * \param context           Tracker context.
 * \param int_ms            Integration period (ms).
 * \param corr_prompt_real  Real part of the prompt correlation.
 */
void tracker_bit_sync_update(tracker_context_t *context, u32 int_ms,
                             s32 corr_prompt_real)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  /* Update bit sync */
  s32 bit_integrate;
  if (bit_sync_update(&internal_data->bit_sync, corr_prompt_real, int_ms,
                      &bit_integrate)) {

    s8 soft_bit = nav_bit_quantize(bit_integrate);

    /* write to FIFO */
    nav_bit_fifo_element_t element = { .soft_bit = soft_bit };
    if (nav_bit_fifo_write(&internal_data->nav_bit_fifo, &element)) {

      /* warn if the FIFO has become full */
      if (nav_bit_fifo_full(&internal_data->nav_bit_fifo)) {
        log_warn_sid(channel_info->sid, "nav bit FIFO full");
      }
    }

    /* clear nav bit TOW offset */
    internal_data->nav_bit_TOW_offset_ms = 0;
  }
}

/** Get the bit length for a tracker channel.
 *
 * \param context     Tracker context.
 *
 * \return Bit length
 */
u8 tracker_bit_length_get(tracker_context_t *context)
{
  const tracker_channel_info_t *channel_info;
    tracker_internal_data_t *internal_data;
    tracker_internal_context_resolve(context, &channel_info, &internal_data);

    return internal_data->bit_sync.bit_length;
}

/** Get the bit alignment state for a tracker channel.
 *
 * \param context     Tracker context.
 *
 * \return true if bit sync has been established and the most recent
 *         integration is bit aligned, false otherwise.
 */
bool tracker_bit_aligned(tracker_context_t *context)
{
  const tracker_channel_info_t *channel_info;
    tracker_internal_data_t *internal_data;
    tracker_internal_context_resolve(context, &channel_info, &internal_data);

  return (internal_data->bit_sync.bit_phase ==
            internal_data->bit_sync.bit_phase_ref);
}

/** Sets a channel's carrier phase ambiguity to unknown.
 * Changes the lock counter to indicate to the consumer of the tracking channel
 * observations that the carrier phase ambiguity may have changed. Also
 * invalidates the half cycle ambiguity, which must be resolved again by the navigation
 *  message processing. Should be called if a cycle slip is suspected.
 *
 * \param context     Tracker context.
 */
void tracker_ambiguity_unknown(tracker_context_t *context)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  internal_data->bit_polarity = BIT_POLARITY_UNKNOWN;
  internal_data->lock_counter =
      tracking_lock_counter_increment(channel_info->sid);
  internal_data->carrier_phase_offset = 0.0;
}

/** Output a correlation data message for a tracker channel.
 *
 * \param context     Tracker context.
 * \param cs          Array of correlations to send.
 */
void tracker_correlations_send(tracker_context_t *context, const corr_t *cs)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  /* Output I/Q correlations using SBP if enabled for this channel */
  if (internal_data->output_iq) {
    msg_tracking_iq_t msg = {
      .channel = channel_info->nap_channel,
    };
    msg.sid = sid_to_sbp(channel_info->sid);
    for (u32 i = 0; i < 3; i++) {
      msg.corrs[i].I = cs[i].I;
      msg.corrs[i].Q = cs[i].Q;
    }
    sbp_send_msg(SBP_MSG_TRACKING_IQ, sizeof(msg), (u8*)&msg);
  }
}

/** \} */

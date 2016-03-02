/*
 * Copyright (C) 2011-2016 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <ch.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>

#include "board/nap/track_channel.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "track.h"
#include "track_api.h"
#include "track_internal.h"
#include "simulator.h"
#include "settings.h"
#include "signal.h"

/** \defgroup tracking Tracking
 * Track satellites via interrupt driven updates to SwiftNAP tracking channels.
 * Initialize SwiftNAP tracking channels. Run loop filters and update
 * channels' code / carrier frequencies each integration period. Update
 * tracking measurements each integration period.
 * \{ */

#define NUM_TRACKER_CHANNELS  12

#define COMPILER_BARRIER() asm volatile ("" : : : "memory")

#define GPS_WEEK_LENGTH_ms (1000 * WEEK_SECS)
#define CHANNEL_SHUTDOWN_TIME_ms 100

typedef enum {
  STATE_DISABLED,
  STATE_ENABLED,
  STATE_DISABLE_REQUESTED
} state_t;

typedef enum {
  EVENT_ENABLE,
  EVENT_DISABLE_REQUEST,
  EVENT_DISABLE
} event_t;

/** Top-level generic tracker channel. */
typedef struct {
  /** State of this channel. */
  state_t state;
  /** Time at which the channel was disabled. */
  systime_t disable_time;
  /** Info associated with this channel. */
  tracker_channel_info_t info;
  /** Data common to all tracker implementations. RW from channel interface
   * functions. RO from functions in this module. */
  tracker_common_data_t common_data;
  /** Data used by the API for all tracker implementations. RW from API
   * functions called within channel interface functions. RO from functions
   * in this module. */
  tracker_internal_data_t internal_data;
  /** Mutex used to protect access to the channel. */
  Mutex mutex;
  /** Mutex used to handle NAP shutdown delay. */
  Mutex nap_mutex;
  /** Elevation angle, degrees. TODO: find a better place for this. */
  s8 elevation;
  /** Associated tracker instance. */
  tracker_t *tracker;
} tracker_channel_t;

static tracker_channel_t tracker_channels[NUM_TRACKER_CHANNELS];

static const tracker_interface_t tracker_interface_default = {
  .code =         CODE_INVALID,
  .init =         0,
  .disable =      0,
  .update =       0,
  .trackers =     0,
  .num_trackers = 0
};

static u16 iq_output_mask = 0;

static void tracker_channel_process(tracker_channel_t *tracker_channel,
                                     bool update_required);

static update_count_t update_count_diff(const tracker_channel_t *
                                        tracker_channel,
                                        const update_count_t *val);
static bool track_iq_output_notify(struct setting *s, const char *val);
static void nap_channel_disable(const tracker_channel_t *tracker_channel);

static tracker_channel_t * tracker_channel_get(tracker_channel_id_t id);
static const tracker_interface_t * tracker_interface_get(gnss_signal_t sid);
static bool tracker_channel_runnable(const tracker_channel_t *tracker_channel,
                                     gnss_signal_t sid, tracker_t **tracker,
                                     const tracker_interface_t **
                                     tracker_interface);
static bool available_tracker_get(const tracker_interface_t *tracker_interface,
                                  tracker_t **tracker);
static state_t tracker_channel_state_get(const tracker_channel_t *
                                         tracker_channel);
static bool tracker_active(const tracker_t *tracker);
static void interface_function(tracker_channel_t *tracker_channel,
                               tracker_interface_function_t func);
static void event(tracker_channel_t *d, event_t event);
static void common_data_init(tracker_common_data_t *common_data,
                             u32 sample_count, float carrier_freq, float cn0);
static void tracker_channel_lock(tracker_channel_t *tracker_channel);
static void tracker_channel_unlock(tracker_channel_t *tracker_channel);


/** Set up the tracking module. */
void track_setup(void)
{
  SETTING_NOTIFY("track", "iq_output_mask", iq_output_mask, TYPE_INT,
                 track_iq_output_notify);

  track_internal_setup();

  for (u32 i=0; i<NUM_TRACKER_CHANNELS; i++) {
    tracker_channels[i].state = STATE_DISABLED;
    tracker_channels[i].tracker = 0;
    chMtxInit(&tracker_channels[i].mutex);
    chMtxInit(&tracker_channels[i].nap_mutex);
  }
}

/** Send tracking state SBP message.
 * Send information on each tracking channel to host.
 */
void tracking_send_state()
{
  tracking_channel_state_t states[nap_track_n_channels];

  if (simulation_enabled_for(SIMULATION_MODE_TRACKING)) {

    u8 num_sats = simulation_current_num_sats();
    for (u8 i=0; i < num_sats; i++) {
      states[i] = simulation_current_tracking_state(i);
    }
    if (num_sats < nap_track_n_channels) {
      for (u8 i = num_sats; i < nap_track_n_channels; i++) {
        states[i].state = 0;
        states[i].sid.code = 0;
        states[i].sid.sat = 0;
        states[i].cn0 = -1;
      }
    }

  } else {

    for (u8 i=0; i<nap_track_n_channels; i++) {

      tracker_channel_t *tracker_channel = tracker_channel_get(i);
      const tracker_common_data_t *common_data = &tracker_channel->common_data;

      bool running;
      gnss_signal_t sid;
      float cn0;

      tracker_channel_lock(tracker_channel);
      {
        running =
            (tracker_channel_state_get(tracker_channel) == STATE_ENABLED);
        sid = tracker_channel->info.sid;
        cn0 = common_data->cn0;
      }
      tracker_channel_unlock(tracker_channel);

      if (!running) {
        states[i].state = 0;
        states[i].sid.code = 0;
        states[i].sid.sat = 0;
        states[i].cn0 = -1;
      } else {
        states[i].state = 1;
        states[i].sid = sid_to_sbp(sid);
        states[i].cn0 = cn0;
      }
    }
  }

  sbp_send_msg(SBP_MSG_TRACKING_STATE, sizeof(states), (u8*)states);
}

/** Calculate the future code phase after N samples.
 * Calculate the expected code phase in N samples time with carrier aiding.
 *
 * \param code_phase   Current code phase in chips.
 * \param carrier_freq Current carrier frequency (i.e. Doppler) in Hz used for
 *                     carrier aiding.
 * \param n_samples    N, the number of samples to propagate for.
 *
 * \return The propagated code phase in chips.
 */
float propagate_code_phase(float code_phase, float carrier_freq, u32 n_samples)
{
  /* Calculate the code phase rate with carrier aiding. */
  u32 code_phase_rate = (1.0 + carrier_freq/GPS_L1_HZ) *
                            NAP_TRACK_NOMINAL_CODE_PHASE_RATE;

  /* Internal Swift NAP code phase is in chips*2^32:
   *
   * |  Chip no.  | Sub-chip | Fractional sub-chip |
   * | 0 ... 1022 | 0 ... 15 |  0 ... (2^28 - 1)   |
   *
   * Code phase rate is directly added in this representation,
   * the nominal code phase rate corresponds to 1 sub-chip.
   */

  /* Calculate code phase in chips*2^32. */
  u64 propagated_code_phase = (u64)(code_phase * (((u64)1)<<32)) + n_samples *
                                  (u64)code_phase_rate;

  /* Convert code phase back to natural units with sub-chip precision.
   * NOTE: the modulo is required to fix the fact rollover should
   * occur at 1023 not 1024.
   */
  return (float)((u32)(propagated_code_phase >> 28) % (1023*16)) / 16.0;
}

/** Handles pending IRQs for the specified tracking channels.
 * \param channels_mask   Bitfield indicating the tracking channels for which
 *                        an IRQ is pending.
 */
void tracking_channels_update(u32 channels_mask)
{
  /* For each tracking channel, call tracking_channel_process(). Indicate
   * that an update is required if the corresponding bit is set in
   * channels_mask.
   */
  for (u32 channel = 0; channel < nap_track_n_channels; channel++) {
    tracker_channel_t *tracker_channel = tracker_channel_get(channel);
    bool update_required = (channels_mask & 1) ? true : false;
    tracker_channel_process(tracker_channel, update_required);
    channels_mask >>= 1;
  }
}

/** Determine if a tracker channel is available to track the specified sid.
 *
 * \param id      ID of the tracker channel to be checked.
 * \param sid     Signal to be tracked.
 *
 * \return true if the tracker channel is available, false otherwise.
 */
bool tracker_channel_available(tracker_channel_id_t id, gnss_signal_t sid)
{
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);

  tracker_t *tracker;
  const tracker_interface_t *tracker_interface;
  return tracker_channel_runnable(tracker_channel, sid, &tracker,
                                  &tracker_interface);
}

/** Initialize a tracker channel to track the specified sid.
 *
 * \param id                    ID of the tracker channel to be initialized.
 * \param sid                   Signal to be tracked.
 * \param carrier_freq          Carrier frequency Doppler (Hz).
 * \param start_sample_count    NAP sample count at which to start the channel.
 * \param cn0_init              Initial C/N0 estimate (dBHz).
 * \param elevation             Elevation (deg).
 *
 * \return true if the tracker channel was initialized, false otherwise.
 */
bool tracker_channel_init(tracker_channel_id_t id, gnss_signal_t sid,
                          float carrier_freq,  u32 start_sample_count,
                          float cn0_init, s8 elevation)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(id);

  const tracker_interface_t *tracker_interface;
  tracker_t *tracker;
  if(!tracker_channel_runnable(tracker_channel, sid, &tracker,
                               &tracker_interface)) {
    return false;
  }

  tracker_channel_lock(tracker_channel);
  {
    /* Set up channel */
    tracker_channel->info.sid = sid;
    tracker_channel->info.context = tracker_channel;
    tracker_channel->info.nap_channel = id;
    tracker_channel->tracker = tracker;

    tracker_channel->elevation = elevation;

    /* Adjust the channel start time as the start_sample_count passed
     * in corresponds to a PROMPT code phase rollover but we want to
     * start the channel on an EARLY code phase rollover.
     */
    /* TODO : change hardcoded sample rate */
    start_sample_count -= 0.5*16;

    common_data_init(&tracker_channel->common_data, start_sample_count,
                     carrier_freq, cn0_init);
    internal_data_init(&tracker_channel->internal_data, sid);
    interface_function(tracker_channel, tracker_interface->init);

    const tracker_common_data_t *common_data = &tracker_channel->common_data;

    /* Lock the NAP mutex while setting up NAP registers and updating state.
     * This allows the update thread to deal with trailing interrupts after
     * the channel is disabled by writing the update register as required. */
    chMtxLock(&tracker_channel->nap_mutex);

    /* Starting carrier phase is set to zero as we don't
     * know the carrier freq well enough to calculate it.
     */
    /* Start with code phase of zero as we have conspired for the
     * channel to be initialised on an EARLY code phase rollover.
     */
    nap_track_code_wr_blocking(tracker_channel->info.nap_channel, sid);
    nap_track_init_wr_blocking(tracker_channel->info.nap_channel, 0, 0, 0);
    nap_track_update_wr_blocking(
      tracker_channel->info.nap_channel,
      common_data->carrier_freq * NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ,
      common_data->code_phase_rate * NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ,
      0, 0
    );

    /* Schedule the timing strobe for start_sample_count. */
    nap_timing_strobe(start_sample_count);

    /* Change the channel state to ENABLED. */
    event(tracker_channel, EVENT_ENABLE);

    Mutex *m = chMtxUnlock();
    assert(m == &tracker_channel->nap_mutex);
  }
  tracker_channel_unlock(tracker_channel);

  return true;
}

/** Disable the specified tracker channel.
 *
 * \param id      ID of the tracker channel to be disabled.
 *
 * \return true if the tracker channel was disabled, false otherwise.
 */
bool tracker_channel_disable(tracker_channel_id_t id)
{
  /* Request disable */
  tracker_channel_t *tracker_channel = tracker_channel_get(id);
  event(tracker_channel, EVENT_DISABLE_REQUEST);
  return true;
}

/** Lock a tracker channel for exclusive access.
 *
 * \note Blocking or long-running operations should not be performed while
 *       holding the lock for a tracking channel.
 *
 * \param id      ID of the tracker channel to be locked.
 */
void tracking_channel_lock(tracker_channel_id_t id)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(id);
  tracker_channel_lock(tracker_channel);
}

/** Unlock a locked tracker channel.
 *
 * \param id      ID of the tracker channel to be unlocked.
 */
void tracking_channel_unlock(tracker_channel_id_t id)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(id);
  tracker_channel_unlock(tracker_channel);
}

/** Determine whether a tracker channel is currently running.
 *
 * \param id      ID of the tracker channel to use.
 */
bool tracking_channel_running(tracker_channel_id_t id)
{
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);
  return (tracker_channel_state_get(tracker_channel) == STATE_ENABLED);
}

/** Return the C/N0 estimate for a tracker channel.
 *
 * \param id      ID of the tracker channel to use.
 */
float tracking_channel_cn0_get(tracker_channel_id_t id)
{
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);
  const tracker_common_data_t *common_data = &tracker_channel->common_data;
  return common_data->cn0;
}

/** Return the time in ms for which a tracker channel has been running.
 *
 * \param id      ID of the tracker channel to use.
 */
u32 tracking_channel_running_time_ms_get(tracker_channel_id_t id)
{
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);
  const tracker_common_data_t *common_data = &tracker_channel->common_data;
  return common_data->update_count;
}

/** Return the time in ms for which C/N0 has been above the use threshold for
 * a tracker channel.
 *
 * \param id      ID of the tracker channel to use.
 */
u32 tracking_channel_cn0_useable_ms_get(tracker_channel_id_t id)
{
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);
  const tracker_common_data_t *common_data = &tracker_channel->common_data;
  return update_count_diff(tracker_channel,
                           &common_data->cn0_below_use_thres_count);
}

/** Return the time in ms for which C/N0 has been below the drop threshold for
 * a tracker channel.
 *
 * \param id      ID of the tracker channel to use.
 */
u32 tracking_channel_cn0_drop_ms_get(tracker_channel_id_t id)
{
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);
  const tracker_common_data_t *common_data = &tracker_channel->common_data;
  return update_count_diff(tracker_channel,
                           &common_data->cn0_above_drop_thres_count);
}

/** Return the time in ms for which the optimistic lock detector has reported
 * being unlocked for a tracker channel.
 *
 * \param id      ID of the tracker channel to use.
 */
u32 tracking_channel_ld_opti_unlocked_ms_get(tracker_channel_id_t id)
{
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);
  const tracker_common_data_t *common_data = &tracker_channel->common_data;
  return update_count_diff(tracker_channel,
                           &common_data->ld_opti_locked_count);
}

/** Return the time in ms for which the pessimistic lock detector has reported
 * being locked for a tracker channel.
 *
 * \param id      ID of the tracker channel to use.
 */
u32 tracking_channel_ld_pess_locked_ms_get(tracker_channel_id_t id)
{
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);
  const tracker_common_data_t *common_data = &tracker_channel->common_data;
  return update_count_diff(tracker_channel,
                           &common_data->ld_pess_unlocked_count);
}

/** Return the time in ms since the last mode change for a tracker channel.
 *
 * \param id      ID of the tracker channel to use.
 */
u32 tracking_channel_last_mode_change_ms_get(tracker_channel_id_t id)
{
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);
  const tracker_common_data_t *common_data = &tracker_channel->common_data;
  return update_count_diff(tracker_channel,
                           &common_data->mode_change_count);
}

/** Return the sid currently associated with a tracker channel.
 *
 * \note The returned sid is only guaranteed to be valid if the tracker
 *       channel is running.
 *
 * \param id      ID of the tracker channel to use.
 */
gnss_signal_t tracking_channel_sid_get(tracker_channel_id_t id)
{
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);
  return tracker_channel->info.sid;
}

/** Return the current carrier frequency for a tracker channel.
 *
 * \param id      ID of the tracker channel to use.
 */
double tracking_channel_carrier_freq_get(tracker_channel_id_t id)
{
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);
  const tracker_common_data_t *common_data = &tracker_channel->common_data;
  return common_data->carrier_freq;
}

/** Return the current time of week for a tracker channel.
 *
 * \param id      ID of the tracker channel to use.
 */
s32 tracking_channel_tow_ms_get(tracker_channel_id_t id)
{
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);
  const tracker_common_data_t *common_data = &tracker_channel->common_data;
  return common_data->TOW_ms;
}

/** Return the bit sync status for a tracker channel.
 *
 * \param id      ID of the tracker channel to use.
 */
bool tracking_channel_bit_sync_resolved(tracker_channel_id_t id)
{
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);
  const tracker_internal_data_t *internal_data =
      &tracker_channel->internal_data;
  return (internal_data->bit_sync.bit_phase_ref != BITSYNC_UNSYNCED);
}

/** Return the bit polarity resolution status for a tracker channel.
 *
 * \param id      ID of the tracker channel to use.
 */
bool tracking_channel_bit_polarity_resolved(tracker_channel_id_t id)
{
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);
  const tracker_internal_data_t *internal_data =
      &tracker_channel->internal_data;
  return (internal_data->bit_polarity != BIT_POLARITY_UNKNOWN);
}

/** Retrieve a channel measurement for a tracker channel.
 *
 * \param id      ID of the tracker channel to use.
 * \param meas    Pointer to output channel_measurement_t.
 */
void tracking_channel_measurement_get(tracker_channel_id_t id,
                                      channel_measurement_t *meas)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(id);
  const tracker_internal_data_t *internal_data =
      &tracker_channel->internal_data;
  const tracker_common_data_t *common_data = &tracker_channel->common_data;

  /* Update our channel measurement. */
  meas->sid = tracker_channel->info.sid;
  meas->code_phase_chips = (double)common_data->code_phase_early /
                               NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP;
  meas->code_phase_rate = common_data->code_phase_rate;
  meas->carrier_phase = common_data->carrier_phase / (double)(1<<24);
  meas->carrier_freq = common_data->carrier_freq;
  meas->time_of_week_ms = common_data->TOW_ms;
  meas->receiver_time = (double)common_data->sample_count / SAMPLE_FREQ;
  meas->snr = common_data->cn0;
  if (internal_data->bit_polarity == BIT_POLARITY_INVERTED) {
    meas->carrier_phase += 0.5;
  }
  meas->lock_counter = internal_data->lock_counter;
}

/** Set the elevation angle for a tracker channel by sid.
 *
 * \param sid         Signal identifier for which the elevation should be set.
 * \param elevation   Elevation angle (deg).
 */
bool tracking_channel_evelation_degrees_set(gnss_signal_t sid, s8 elevation)
{
  bool result = false;
  for (u32 i=0; i < NUM_TRACKER_CHANNELS; i++) {
    tracker_channel_t *tracker_channel = tracker_channel_get(i);

    /* Check SID before locking. */
    if (!sid_is_equal(tracker_channel->info.sid, sid)) {
      continue;
    }

    /* Lock and update if SID matches. */
    tracker_channel_lock(tracker_channel);
    {
      if (sid_is_equal(tracker_channel->info.sid, sid)) {
        tracker_channel->elevation = elevation;
        result = true;
      }
    }
    tracker_channel_unlock(tracker_channel);
    break;
  }
  return result;
}

/** Return the elevation angle for a tracker channel.
 *
 * \param id      ID of the tracker channel to use.
 */
s8 tracking_channel_evelation_degrees_get(tracker_channel_id_t id)
{
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);
  return tracker_channel->elevation;
}

/** Read the next pending nav bit for a tracker channel.
 *
 * \note This function should should be called from the same thread as
 * tracking_channel_time_sync().
 *
 * \param id            ID of the tracker channel to read from.
 * \param soft_bit      Output soft nav bit value.
 *
 * \return true if *soft_bit is valid, false otherwise.
 */
bool tracking_channel_nav_bit_get(tracker_channel_id_t id, s8 *soft_bit)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(id);
  tracker_internal_data_t *internal_data = &tracker_channel->internal_data;

  nav_bit_fifo_element_t element;
  if (nav_bit_fifo_read(&internal_data->nav_bit_fifo, &element)) {
    *soft_bit = element.soft_bit;
    return true;
  }
  return false;
}

/** Propagate decoded time of week and bit polarity back to a tracker channel.
 *
 * \note This function should be called from the same thread as
 * tracking_channel_nav_bit_get().
 * \note It is assumed that the specified data is synchronized with the most
 * recent nav bit read from the FIFO using tracking_channel_nav_bit_get().
 *
 * \param id            ID of the tracker channel to synchronize.
 * \param TOW_ms        Time of week in milliseconds.
 * \param bit_polarity  Bit polarity.
 *
 * \return true if data was enqueued successfully, false otherwise.
 */
bool tracking_channel_time_sync(tracker_channel_id_t id, s32 TOW_ms,
                                s8 bit_polarity)
{
  assert(TOW_ms >= 0);
  assert(TOW_ms < GPS_WEEK_LENGTH_ms);
  assert((bit_polarity == BIT_POLARITY_NORMAL) ||
         (bit_polarity == BIT_POLARITY_INVERTED));

  tracker_channel_t *tracker_channel = tracker_channel_get(id);
  tracker_internal_data_t *internal_data = &tracker_channel->internal_data;
  nav_bit_fifo_index_t read_index = internal_data->nav_bit_fifo.read_index;
  return nav_time_sync_set(&internal_data->nav_time_sync,
                           TOW_ms, bit_polarity, read_index);
}

/** Retrieve the channel info and internal data associated with a
 * tracker context.
 *
 * \note This function is declared in track_internal.h to avoid polluting
 * the public API in track.h
 *
 * \param tracker_context     Tracker context to be resolved.
 * \param channel_info        Output tracker channel info.
 * \param internal_data       Output tracker internal data.
 */
void tracker_internal_context_resolve(tracker_context_t *tracker_context,
                                      const tracker_channel_info_t **channel_info,
                                      tracker_internal_data_t **internal_data)
{
  tracker_channel_t *tracker_channel = (tracker_channel_t *)tracker_context;
  *channel_info = &tracker_channel->info;
  *internal_data = &tracker_channel->internal_data;
}

/** Check the state of a tracker channel and generate events as required.
 * \param tracker_channel   Tracker channel to use.
 * \param update_required   True when correlations are pending for the
 *                          tracking channel.
 */
static void tracker_channel_process(tracker_channel_t *tracker_channel,
                                    bool update_required)
{
  switch (tracker_channel_state_get(tracker_channel)) {
    case STATE_ENABLED:
    {
      if (update_required) {
        const tracker_interface_t *tracker_interface =
            tracker_interface_get(tracker_channel->info.sid);
        tracker_channel_lock(tracker_channel);
        {
          interface_function(tracker_channel, tracker_interface->update);
        }
        tracker_channel_unlock(tracker_channel);
      }
      break;
    }
    case STATE_DISABLE_REQUESTED:
    {
      const tracker_interface_t *tracker_interface =
          tracker_interface_get(tracker_channel->info.sid);
      nap_channel_disable(tracker_channel);
      tracker_channel_lock(tracker_channel);
      {
        interface_function(tracker_channel, tracker_interface->disable);
        tracker_channel->disable_time = chTimeNow();
        event(tracker_channel, EVENT_DISABLE);
      }
      tracker_channel_unlock(tracker_channel);
      break;
    }
    case STATE_DISABLED:
    {
      if (update_required) {
        /* Tracking channel is not owned by the update thread, but the update
         * register must be written to clear the interrupt flag. Atomically
         * verify state and write the update register. */
        chMtxLock(&tracker_channel->nap_mutex);
        if (tracker_channel_state_get(tracker_channel) == STATE_DISABLED) {
          nap_channel_disable(tracker_channel);
        }
        Mutex *m = chMtxUnlock();
        assert(m == &tracker_channel->nap_mutex);
      }
      break;
    }
    default:
    {
      assert(!"Invalid tracking channel state");
      break;
    }
  }
}

/** Return the unsigned difference between update_count and *val for a
 * tracker channel.
 *
 * \note This function allows some margin to avoid glitches in case values
 * are not read atomically from the tracking channel data.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param val               Pointer to the value to be subtracted
 *                          from update_count.
 *
 * \return The unsigned difference between update_count and *val.
 */
static update_count_t update_count_diff(const tracker_channel_t *
                                        tracker_channel,
                                        const update_count_t *val)
{
  const tracker_common_data_t *common_data = &tracker_channel->common_data;
  update_count_t result = (update_count_t)(common_data->update_count - *val);
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  /* Allow some margin in case values were not read atomically.
   * Treat a difference of [-10000, 0) as zero. */
  if (result > (update_count_t)(UINT32_MAX - 10000))
    return 0;
  else
    return result;
}

/** Parse the IQ output enable bitfield. */
static bool track_iq_output_notify(struct setting *s, const char *val)
{
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    for (int i = 0; i < NUM_TRACKER_CHANNELS; i++) {
      tracker_channel_t *tracker_channel = tracker_channel_get(i);
      tracker_internal_data_t *internal_data = &tracker_channel->internal_data;
      internal_data->output_iq = (iq_output_mask & (1 << i)) != 0;
    }
    return true;
  }
  return false;
}

/** Disable the NAP tracking channel associated with a tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 */
static void nap_channel_disable(const tracker_channel_t *tracker_channel)
{
  nap_track_update_wr_blocking(tracker_channel->info.nap_channel, 0, 0, 0, 0);
}

/** Retrieve the tracker channel associated with a tracker channel ID.
 *
 * \param tracker_channel_id    ID of the tracker channel to be retrieved.
 *
 * \return Associated tracker channel.
 */
static tracker_channel_t * tracker_channel_get(tracker_channel_id_t id)
{
  assert(id < NUM_TRACKER_CHANNELS);
  return &tracker_channels[id];
}

/** Retrieve the tracker interface for the specified sid.
 *
 * \param sid       Signal to be tracked.
 *
 * \return Associated tracker interface. May be the default interface.
 */
static const tracker_interface_t * tracker_interface_get(gnss_signal_t sid)
{
  const tracker_interface_list_element_t *e = *tracker_interface_list_ptr_get();
  while (e != 0) {
    const tracker_interface_t *interface = e->interface;
    if (interface->code == sid.code) {
      return interface;
    }
    e = e->next;
  }

  return &tracker_interface_default;
}

/** Determine if a tracker channel can be started to track the specified sid.
 *
 * \param tracker_channel_id    ID of the tracker channel to be checked.
 * \param sid                   Signal to be tracked.
 * \param tracker_interface     Output tracker interface to use.
 * \param tracker               Output tracker instance to use.
 *
 * \return true if the tracker channel is available, false otherwise.
 */
static bool tracker_channel_runnable(const tracker_channel_t *tracker_channel,
                                     gnss_signal_t sid, tracker_t **tracker,
                                     const tracker_interface_t **
                                     tracker_interface)
{
  if (tracker_channel_state_get(tracker_channel) != STATE_DISABLED)
      return false;
  if (chTimeElapsedSince(tracker_channel->disable_time) <
        MS2ST(CHANNEL_SHUTDOWN_TIME_ms))
    return false;

  *tracker_interface = tracker_interface_get(sid);
  if (!available_tracker_get(*tracker_interface, tracker))
    return false;

  return true;
}

/** Find an inactive tracker instance for the specified tracker interface.
 *
 * \param tracker_interface   Tracker interface to use.
 * \param tracker             Output inactive tracker instance.
 *
 * \return true if *tracker points to an inactive tracker instance,
 * false otherwise.
 */
static bool available_tracker_get(const tracker_interface_t *tracker_interface,
                                  tracker_t **tracker)
{
  /* Search for a free tracker */
  for (u32 i=0; i<tracker_interface->num_trackers; i++) {
    tracker_t *t = &tracker_interface->trackers[i];
    if (!tracker_active(t)) {
      *tracker = t;
      return true;
    }
  }

  return false;
}

/** Return the state of a tracker channel.
 *
 * \note This function performs an acquire operation, meaning that it ensures
 * the returned state was read before any subsequent memory accesses.
 *
 * \param tracker_channel   Tracker channel to use.
 *
 * \return state of the decoder channel.
 */
static state_t tracker_channel_state_get(const tracker_channel_t *
                                         tracker_channel)
{
  state_t state = tracker_channel->state;
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  return state;
}

/** Return the state of a tracker instance.
 *
 * \note This function performs an acquire operation, meaning that it ensures
 * the returned state was read before any subsequent memory accesses.
 *
 * \param tracker   Tracker to use.
 *
 * \return true if the tracker is active, false if inactive.
 */
static bool tracker_active(const tracker_t *tracker)
{
  bool active = tracker->active;
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  return active;
}

/** Execute an interface function on a tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param func              Interface function to execute.
 */
static void interface_function(tracker_channel_t *tracker_channel,
                               tracker_interface_function_t func)
{
  func(&tracker_channel->info, &tracker_channel->common_data,
       tracker_channel->tracker->data);
}

/** Update the state of a tracker channel and its associated tracker instance.
 *
 * \note This function performs a release operation, meaning that it ensures
 * all prior memory accesses have completed before updating state information.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param event             Event to process.
 */
static void event(tracker_channel_t *tracker_channel, event_t event)
{
  switch (event) {
  case EVENT_ENABLE: {
    assert(tracker_channel->state == STATE_DISABLED);
    assert(tracker_channel->tracker->active == false);
    tracker_channel->tracker->active = true;
    /* Sequence point for enable is setting channel state = STATE_ENABLED */
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    tracker_channel->state = STATE_ENABLED;
  }
  break;

  case EVENT_DISABLE_REQUEST: {
    assert(tracker_channel->state == STATE_ENABLED);
    tracker_channel->state = STATE_DISABLE_REQUESTED;
  }
  break;

  case EVENT_DISABLE: {
    assert(tracker_channel->state == STATE_DISABLE_REQUESTED);
    assert(tracker_channel->tracker->active == true);
    /* Sequence point for disable is setting channel state = STATE_DISABLED
     * and/or tracker active = false (order of these two is irrelevant here) */
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    tracker_channel->tracker->active = false;
    tracker_channel->state = STATE_DISABLED;
  }
  break;
  }
}

/** Initialize a tracker common data structure.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param sample_count      Sample count.
 * \param carrier_freq      Carrier frequency.
 * \param cn0               C/N0 estimate.
 */
static void common_data_init(tracker_common_data_t *common_data,
                             u32 sample_count, float carrier_freq, float cn0)
{
  /* Initialize all fields to 0 */
  memset(common_data, 0, sizeof(tracker_common_data_t));

  common_data->TOW_ms = TOW_INVALID;

  /* Calculate code phase rate with carrier aiding. */
  common_data->code_phase_rate = (1 + carrier_freq/GPS_L1_HZ) *
                                    GPS_CA_CHIPPING_RATE;
  common_data->carrier_freq = carrier_freq;

  common_data->sample_count = sample_count;
  common_data->cn0 = cn0;
}

/** Lock a tracker channel for exclusive access.
 *
 * \param tracker_channel   Tracker channel to use.
 */
static void tracker_channel_lock(tracker_channel_t *tracker_channel)
{
  chMtxLock(&tracker_channel->mutex);
}

/** Unlock a locked tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 */
static void tracker_channel_unlock(tracker_channel_t *tracker_channel)
{
  Mutex *m = chMtxUnlock();
  assert(m == &tracker_channel->mutex);
}

/** \} */

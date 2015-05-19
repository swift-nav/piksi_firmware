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

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "board/nap/track_channel.h"
#include "sbp.h"
#include "track.h"
#include "simulator.h"
#include "peripherals/random.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>

#define LONG_INTEGRATION_INTERVAL 20
#define LONG_INTEGRATION_SNR_THRESH 36 /* C/N0 */

/** \defgroup tracking Tracking
 * Track satellites via interrupt driven updates to SwiftNAP tracking channels.
 * Initialize SwiftNAP tracking channels. Run loop filters and update
 * channels' code / carrier frequencies each integration period. Update
 * tracking measurements each integration period.
 * \{ */

/* Initialiser using GNU extension, see
 * http://gcc.gnu.org/onlinedocs/gcc/Designated-Inits.html
 * tracking_channel_t tracking_channel[NAP_MAX_N_TRACK_CHANNELS] = \
 *   {[0 ... nap_track_n_channels-1] = {.state = TRACKING_DISABLED}};
 */

/* Initialiser actually not needed, static array inits to zero
 * and TRACKING_DISABLED = 0.
 */
tracking_channel_t tracking_channel[NAP_MAX_N_TRACK_CHANNELS];

/* PRN lock counter
 * A map of PRN to an initially random number that increments each time that
 * PRN begins being tracked.
 */
static u16 tracking_lock_counters[MAX_SATS];

/* Initialize the lock counters to random numbers
 */
void initialize_lock_counters(void) {
  rng_setup();
  for (u8 i=0; i < MAX_SATS; i++) {
    tracking_lock_counters[i] = random_int();
  }
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
  u32 code_phase_rate = (1.0 + carrier_freq/GPS_L1_HZ) * NAP_TRACK_NOMINAL_CODE_PHASE_RATE;

  /* Internal Swift NAP code phase is in chips*2^32:
   *
   * |  Chip no.  | Sub-chip | Fractional sub-chip |
   * | 0 ... 1022 | 0 ... 15 |  0 ... (2^28 - 1)   |
   *
   * Code phase rate is directly added in this representation,
   * the nominal code phase rate corresponds to 1 sub-chip.
   */

  /* Calculate code phase in chips*2^32. */
  u64 propagated_code_phase = (u64)(code_phase * (((u64)1)<<32)) + n_samples * (u64)code_phase_rate;

  /* Convert code phase back to natural units with sub-chip precision.
   * NOTE: the modulo is required to fix the fact rollover should
   * occur at 1023 not 1024.
   */
  return (float)((u32)(propagated_code_phase >> 28) % (1023*16)) / 16.0;
}

/** Initialises a tracking channel.
 * Initialises a tracking channel on the Swift NAP. The start_sample_count
 * must be contrived to be at or close to a PRN edge (PROMPT code phase = 0).
 *
 * \param prn                PRN number - 1 (0-31).
 * \param channel            Tracking channel number on the Swift NAP.
 * \param carrier_freq       Carrier frequency (Doppler) at start of tracking in Hz.
 * \param start_sample_count Sample count on which to start tracking.
 */
void tracking_channel_init(u8 channel, u8 prn, float carrier_freq,
                           u32 start_sample_count, float snr)
{
  /* Calculate code phase rate with carrier aiding. */
  float code_phase_rate = (1 + carrier_freq/GPS_L1_HZ) * GPS_CA_CHIPPING_RATE;
  tracking_channel_t *chan = &tracking_channel[channel];

  /* Adjust the channel start time as the start_sample_count passed
   * in corresponds to a PROMPT code phase rollover but we want to
   * start the channel on an EARLY code phase rollover.
   */
  /* TODO : change hardcoded sample rate */
  start_sample_count -= 0.5*16;

  /* Setup tracking_channel struct. */
  chan->state = TRACKING_RUNNING;
  chan->prn = prn;
  chan->update_count = 0;

  /* Initialize TOW_ms and lock_count. */
  tracking_channel_ambiguity_unknown(channel);

  chan->snr_above_threshold_count = 0;
  chan->snr_below_threshold_count = 0;

  aided_tl_init(&(chan->tl_state), 1e3,
                code_phase_rate-1.023e6, 1, 0.7, 1,
                carrier_freq, 25, 0.7, 1,
                5);

  chan->code_phase_early = 0;
  chan->code_phase_rate_fp = code_phase_rate*NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;
  chan->code_phase_rate_fp_prev = chan->code_phase_rate_fp;
  chan->code_phase_rate = code_phase_rate;
  chan->carrier_phase = 0;
  chan->carrier_freq = carrier_freq;
  chan->carrier_freq_fp = (s32)(carrier_freq * NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ);
  chan->carrier_freq_fp_prev = chan->carrier_freq_fp;
  chan->sample_count = start_sample_count;

  nav_msg_init(&chan->nav_msg);

  chan->int_ms = 1;
  chan->short_cycle = true;

  /* Initialise C/N0 estimator */
  float cn0 = 10 * log10(snr);
  cn0 += 10 * log10(1000); /* Bandwidth */
  cn0_est_init(&chan->cn0_est, 1e3, cn0, 5, 1e3);

  /*alias_detect_init(&chan->alias_detect, 50,
                    (LONG_INTEGRATION_INTERVAL-1)*1e-3);*/

  /* Starting carrier phase is set to zero as we don't
   * know the carrier freq well enough to calculate it.
   */
  /* Start with code phase of zero as we have conspired for the
   * channel to be initialised on an EARLY code phase rollover.
   */
  nap_track_code_wr_blocking(channel, prn);
  nap_track_init_wr_blocking(channel, prn, 0, 0);
  nap_track_update_wr_blocking(
    channel,
    carrier_freq*NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ,
    chan->code_phase_rate_fp,
    0, 0
  );

  /* Schedule the timing strobe for start_sample_count. */
  nap_timing_strobe(start_sample_count);
}

/** Get correlations from a NAP tracking channel and store them in the
 * tracking channel state struct.
 * \param channel Tracking channel to read correlations for.
 */
void tracking_channel_get_corrs(u8 channel)
{
  tracking_channel_t* chan = &tracking_channel[channel];

  switch(chan->state)
  {
    case TRACKING_RUNNING:
      /* Read early ([0]), prompt ([1]) and late ([2]) correlations. */
      if ((chan->int_ms > 1) && !chan->short_cycle) {
        /* If we just requested the short cycle, this is the long cycle's
         * correlations. */
        corr_t cs[3];
        nap_track_corr_rd_blocking(channel, &chan->corr_sample_count, cs);
        /* accumulate short cycle correlations with long */
        for(int i = 0; i < 3; i++) {
          chan->cs[i].I += cs[i].I;
          chan->cs[i].Q += cs[i].Q;
        }
      } else {
        nap_track_corr_rd_blocking(channel, &chan->corr_sample_count, chan->cs);
        alias_detect_first(&chan->alias_detect, chan->cs[1].I, chan->cs[1].Q);
      }
      break;

    case TRACKING_DISABLED:
    default:
      /* TODO: WTF? */
      break;
  }
}

/** Update tracking channels after the end of an integration period.
 * Update update_count, sample_count, TOW, run loop filters and update
 * SwiftNAP tracking channel frequencies.
 * \param channel Tracking channel to update.
 */
void tracking_channel_update(u8 channel)
{
  tracking_channel_t* chan = &tracking_channel[channel];

  switch(chan->state)
  {
    case TRACKING_RUNNING:
    {
      chan->sample_count += chan->corr_sample_count;
      chan->code_phase_early = (u64)chan->code_phase_early +
                               (u64)chan->corr_sample_count
                                 * chan->code_phase_rate_fp_prev;
      chan->carrier_phase += (s64)chan->carrier_freq_fp_prev
                               * chan->corr_sample_count;
      /* TODO: Fix this in the FPGA - first integration is one sample short. */
      if (chan->update_count == 0)
        chan->carrier_phase -= chan->carrier_freq_fp_prev;
      chan->code_phase_rate_fp_prev = chan->code_phase_rate_fp;
      chan->carrier_freq_fp_prev = chan->carrier_freq_fp;

      /* TODO: check TOW_ms = 0 case is correct, 0 is a valid TOW. */
      if (chan->TOW_ms > 0) {
        /* Have a valid time of week. */
        chan->TOW_ms += chan->short_cycle ? 1 : (chan->int_ms-1);
        chan->TOW_ms %= 7*24*60*60*1000;
      }

     if (chan->int_ms > 1) {
        /* If we're doing long integrations alternate between short and long
         * cycles.  This is because of FPGA pipelining and latency.  The
         * loop parameters can only be updated at the end of the second
         * integration interval and waiting a whole 20ms is too long.
         */
        chan->short_cycle = !chan->short_cycle;

        if (!chan->short_cycle) {
          nap_track_update_wr_blocking(
            channel,
            chan->carrier_freq_fp,
            chan->code_phase_rate_fp,
            0, 0
          );
          return;
        }
      }

      chan->update_count += chan->int_ms;


      /* TODO: check TOW_ms = 0 case is correct, 0 is a valid TOW. */
      s32 TOW_ms = nav_msg_update(&chan->nav_msg, chan->cs[1].I, chan->int_ms);

      if (TOW_ms > 0 && chan->TOW_ms != TOW_ms) {
        if (chan->TOW_ms > 0) {
          log_warn("PRN %d TOW mismatch: %ld, %lu\n",
                   chan->prn+1, chan->TOW_ms, TOW_ms);
        }
        chan->TOW_ms = TOW_ms;
      }

      /* Correlations should already be in chan->cs thanks to
       * tracking_channel_get_corrs. */
      corr_t* cs = chan->cs;

      /* Update C/N0 estimate */
      chan->cn0 = cn0_est(&chan->cn0_est, cs[1].I/chan->int_ms);

      /* Run the loop filters. */

      /* TODO: Make this more elegant. */
      correlation_t cs2[3];
      for (u32 i = 0; i < 3; i++) {
        cs2[i].I = cs[2-i].I;
        cs2[i].Q = cs[2-i].Q;
      }
      aided_tl_update(&(chan->tl_state), cs2);
      chan->carrier_freq = chan->tl_state.carr_freq;
      chan->code_phase_rate = chan->tl_state.code_freq + 1.023e6;

      chan->code_phase_rate_fp_prev = chan->code_phase_rate_fp;
      chan->code_phase_rate_fp = chan->code_phase_rate
        * NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;

      chan->carrier_freq_fp = chan->carrier_freq
        * NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ;

      if (chan->int_ms != 1) {
        s32 I = (cs[1].I - chan->alias_detect.first_I) / (LONG_INTEGRATION_INTERVAL-1);
        s32 Q = (cs[1].Q - chan->alias_detect.first_Q) / (LONG_INTEGRATION_INTERVAL-1);
        float err = alias_detect_second(&chan->alias_detect, I, Q);
        if (fabs(err) > 10) {
          log_warn("False phase lock detect PRN%d: err=%f\n", chan->prn+1, err);
          chan->tl_state.carr_freq += err;
          chan->tl_state.carr_filt.y = chan->tl_state.carr_freq;
        }
      }

      if ((chan->TOW_ms > 0) && (chan->int_ms == 1) &&
          (chan->nav_msg.bit_phase == chan->nav_msg.bit_phase_ref)) {
        /* If we have TOW, transition to longer integration. */
        /*log_info("Increasing integration time for PRN %d\n", chan->prn+1);*/
        chan->int_ms = LONG_INTEGRATION_INTERVAL;
        chan->short_cycle = true;

        cn0_est_init(&chan->cn0_est, 1e3/chan->int_ms, chan->cn0, 5, 1e3);

        /* Recalculate filter coefficients: now pure PLL */
        aided_tl_init(&chan->tl_state, 1e3 / chan->int_ms,
                      chan->tl_state.code_freq, 1, 0.7, 1,
                      chan->tl_state.carr_freq, 10, 0.7, 1,
                      0);

        nav_msg_init(&chan->nav_msg);

        alias_detect_init(&chan->alias_detect, 50,
                          (LONG_INTEGRATION_INTERVAL-1)*1e-3);

      } else if ((chan->int_ms !=1 ) &&
                 (chan->cn0 < LONG_INTEGRATION_SNR_THRESH)) {
        /* If we are in long integration and SNR drops too far,
         * return to short integration.
         */
        /*log_info("Decreasing integration time for PRN %d\n", chan->prn+1);*/

        chan->int_ms = 1;
        chan->short_cycle = true;

        aided_tl_init(&chan->tl_state, 1e3,
                      chan->tl_state.code_freq, 1, 0.7, 1,
                      chan->tl_state.carr_freq, 25, 0.7, 1,
                      5);

        cn0_est_init(&chan->cn0_est, 1e3, chan->cn0, 5, 1e3);

      }

      nap_track_update_wr_blocking(
        channel,
        chan->carrier_freq_fp,
        chan->code_phase_rate_fp,
        chan->int_ms == 1 ? 0 : chan->int_ms - 2, 0
      );

      break;
    }
    case TRACKING_DISABLED:
    default:
      /* TODO: WTF? */
      tracking_channel_disable(channel);
      break;
  }
}

/** Disable tracking channel.
 * Change tracking channel state to TRACKING_DISABLED and write 0 to SwiftNAP
 * tracking channel code / carrier frequencies to stop channel from raising
 * interrupts.
 *
 * \param channel Tracking channel to disable.
 */
void tracking_channel_disable(u8 channel)
{
  nap_track_update_wr_blocking(channel, 0, 0, 0, 0);
  tracking_channel[channel].state = TRACKING_DISABLED;
}

/** Sets a channel's carrier phase ambiguity to unknown.
 * Changes the lock counter to indicate to the consumer of the tracking channel
 * observations that the carrier phase ambiguity may have changed. Also
 * invalidates the time of week to indicate that the half cycle ambiguity must
 * be resolved again by the navigation message processing. Should be called if
 * a cycle slip is suspected.
 *
 * \param channel Tracking channel to disable.
 */
void tracking_channel_ambiguity_unknown(u8 channel)
{
  u8 prn = tracking_channel[channel].prn;
  tracking_channel[channel].TOW_ms = TOW_INVALID;
  tracking_channel[channel].lock_counter = ++tracking_lock_counters[prn];
}

/** Update channel measurement for a tracking channel.
 * \param channel Tracking channel to update measurement from.
 * \param meas Pointer to channel_measurement_t where measurement will be put.
 */
void tracking_update_measurement(u8 channel, channel_measurement_t *meas)
{
  tracking_channel_t* chan = &tracking_channel[channel];

  /* Update our channel measurement. */
  meas->prn = chan->prn;
  meas->code_phase_chips = (double)chan->code_phase_early / NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP;
  meas->code_phase_rate = chan->code_phase_rate;
  meas->carrier_phase = chan->carrier_phase / (double)(1<<24);
  meas->carrier_freq = chan->carrier_freq;
  meas->time_of_week_ms = chan->TOW_ms;
  meas->receiver_time = (double)chan->sample_count / SAMPLE_FREQ;
  meas->snr = tracking_channel_snr(channel);
  if (chan->nav_msg.inverted) {
    meas->carrier_phase += 0.5;
  }
  meas->lock_counter = chan->lock_counter;
}

/** Calculate a tracking channel's current SNR.
 * \param channel Tracking channel to calculate SNR of.
 */
float tracking_channel_snr(u8 channel)
{
  return tracking_channel[channel].cn0;
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
        states[i].state = TRACKING_DISABLED;
        states[i].prn   = 0;
        states[i].cn0   = -1;
      }
    }

  } else {

    for (u8 i=0; i<nap_track_n_channels; i++) {
      states[i].state = tracking_channel[i].state;
      states[i].prn = tracking_channel[i].prn;
      if (tracking_channel[i].state == TRACKING_RUNNING)
        states[i].cn0 = tracking_channel_snr(i);
      else
        states[i].cn0 = -1;
    }

  }

  sbp_send_msg(SBP_MSG_TRACKING_STATE, sizeof(states), (u8*)states);

}

/** \} */

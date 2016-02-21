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
#include <assert.h>
#include <ch.h>

#include "board/nap/track_channel.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "track.h"
#include "simulator.h"
#include "peripherals/random.h"
#include "settings.h"
#include "signal.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>

/*  code: nbw zeta k carr_to_code
 carrier:                    nbw  zeta k fll_aid */
#define LOOP_PARAMS_SLOW \
  "(1 ms, (1, 0.7, 1, 1540), (10, 0.7, 1, 5))," \
 "(20 ms, (1, 0.7, 1, 1540), (12, 0.7, 1, 0))"

#define LOOP_PARAMS_MED \
  "(1 ms, (1, 0.7, 1, 1540), (10, 0.7, 1, 5))," \
  "(5 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 0))"

#define LOOP_PARAMS_FAST \
  "(1 ms, (1, 0.7, 1, 1540), (40, 0.7, 1, 5))," \
  "(4 ms, (1, 0.7, 1, 1540), (62, 0.7, 1, 0))"

#define LOOP_PARAMS_EXTRAFAST \
  "(1 ms, (1, 0.7, 1, 1540), (50, 0.7, 1, 5))," \
  "(2 ms, (1, 0.7, 1, 1540), (100, 0.7, 1, 0))"


/*                          k1,   k2,  lp,  lo */
#define LD_PARAMS_PESS     "0.10, 1.4, 200, 50"
#define LD_PARAMS_NORMAL   "0.05, 1.4, 150, 50"
#define LD_PARAMS_OPT      "0.02, 1.1, 150, 50"
#define LD_PARAMS_EXTRAOPT "0.02, 0.8, 150, 50"
#define LD_PARAMS_DISABLE  "0.02, 1e-6, 1, 1"

char loop_params_string[120] = LOOP_PARAMS_MED;
char lock_detect_params_string[24] = LD_PARAMS_DISABLE;
bool use_alias_detection = true;

#define CN0_EST_LPF_CUTOFF 5
#define GPS_WEEK_LENGTH_ms (1000 * WEEK_SECS)

#define NAV_BIT_FIFO_INDEX_MASK ((NAV_BIT_FIFO_SIZE) - 1)
#define NAV_BIT_FIFO_INDEX_DIFF(write_index, read_index) \
          ((nav_bit_fifo_index_t)((write_index) - (read_index)))
#define NAV_BIT_FIFO_LENGTH(p_fifo) \
          (NAV_BIT_FIFO_INDEX_DIFF((p_fifo)->write_index, (p_fifo)->read_index))

static struct loop_params {
  float code_bw, code_zeta, code_k, carr_to_code;
  float carr_bw, carr_zeta, carr_k, carr_fll_aid_gain;
  u8 coherent_ms;
} loop_params_stage[2];

static struct lock_detect_params {
  float k1, k2;
  u16 lp, lo;
} lock_detect_params;

static float track_cn0_use_thres = 31.0; /* dBHz */
static float track_cn0_drop_thres = 31.0;
static u16 iq_output_mask = 0;

/** \defgroup tracking Tracking
 * Track satellites via interrupt driven updates to SwiftNAP tracking channels.
 * Initialize SwiftNAP tracking channels. Run loop filters and update
 * channels' code / carrier frequencies each integration period. Update
 * tracking measurements each integration period.
 * \{ */

/* Initialiser not needed, static array inits to zero
 * and TRACKING_DISABLED = 0.
 */
tracking_channel_t tracking_channel[NAP_MAX_N_TRACK_CHANNELS];

/* signal lock counter
 * A map of signal to an initially random number that increments each time that
 * signal begins being tracked.
 */
static u16 tracking_lock_counters[PLATFORM_SIGNAL_COUNT];

static MUTEX_DECL(nav_time_sync_mutex);

static void tracking_channel_get_corrs(u8 channel);

static void nav_bit_fifo_init(nav_bit_fifo_t *fifo);
static bool nav_bit_fifo_write(nav_bit_fifo_t *fifo,
                               const nav_bit_fifo_element_t *element);
static bool nav_bit_fifo_read(nav_bit_fifo_t *fifo,
                              nav_bit_fifo_element_t *element);
static void nav_time_sync_init(nav_time_sync_t *sync);
static bool nav_time_sync_set(nav_time_sync_t *sync, s32 TOW_ms,
                              s8 bit_polarity, nav_bit_fifo_index_t read_index);
static bool nav_time_sync_get(nav_time_sync_t *sync, s32 *TOW_ms,
                              s8 *bit_polarity, nav_bit_fifo_index_t *read_index);

/** Initialize a nav_bit_fifo_t struct. */
static void nav_bit_fifo_init(nav_bit_fifo_t *fifo)
{
  fifo->read_index = 0;
  fifo->write_index = 0;
}

/** Write data to the nav bit FIFO.
 *
 * \note This function should only be called internally by the tracking thread.
 *
 * \param fifo        nav_bit_fifo_t struct to use.
 * \param element     Element to write to the FIFO.
 *
 * \return true if element was read, false otherwise.
 */
static bool nav_bit_fifo_write(nav_bit_fifo_t *fifo,
                               const nav_bit_fifo_element_t *element)
{
  if (NAV_BIT_FIFO_LENGTH(fifo) < NAV_BIT_FIFO_SIZE) {
    asm volatile ("" : : : "memory"); /* Prevent compiler reordering */
    memcpy(&fifo->elements[fifo->write_index & NAV_BIT_FIFO_INDEX_MASK],
           element, sizeof(nav_bit_fifo_element_t));
    asm volatile ("" : : : "memory"); /* Prevent compiler reordering */
    fifo->write_index++;
    return true;
  }

  return false;
}

/** Read pending data from the nav bit FIFO.
 *
 * \note This function should only be called externally by the decoder thread.
 *
 * \param fifo        nav_bit_fifo_t struct to use.
 * \param element     Output element read from the FIFO.
 *
 * \return true if element was read, false otherwise.
 */
static bool nav_bit_fifo_read(nav_bit_fifo_t *fifo,
                              nav_bit_fifo_element_t *element)
{
  if (NAV_BIT_FIFO_LENGTH(fifo) > 0) {
    asm volatile ("" : : : "memory"); /* Prevent compiler reordering */
    memcpy(element, &fifo->elements[fifo->read_index & NAV_BIT_FIFO_INDEX_MASK],
           sizeof(nav_bit_fifo_element_t));
    asm volatile ("" : : : "memory"); /* Prevent compiler reordering */
    fifo->read_index++;
    return true;
  }

  return false;
}

/** Initialize a nav_time_sync_t struct. */
static void nav_time_sync_init(nav_time_sync_t *sync)
{
  sync->valid = false;
}

/** Write pending time sync data from the decoder thread.
 *
 * \note This function should only be called externally by the decoder thread.
 *
 * \param sync          nav_time_sync_t struct to use.
 * \param TOW_ms        TOW in ms.
 * \param bit_polarity  Bit polarity.
 * \param read_index    Nav bit FIFO read index to which the above values
 *                      are synchronized.
 *
 * \return true if data was stored successfully, false otherwise.
 */
static bool nav_time_sync_set(nav_time_sync_t *sync, s32 TOW_ms,
                              s8 bit_polarity, nav_bit_fifo_index_t read_index)
{
  bool result = false;
  chMtxLock(&nav_time_sync_mutex);

  sync->TOW_ms = TOW_ms;
  sync->bit_polarity = bit_polarity;
  sync->read_index = read_index;
  sync->valid = true;
  result = true;

  Mutex *m = chMtxUnlock();
  assert(m == &nav_time_sync_mutex);
  return result;
}

/** Read pending time sync data provided by the decoder thread.
 *
 * \note This function should only be called internally by the tracking thread.
 *
 * \param sync          nav_time_sync_t struct to use.
 * \param TOW_ms        TOW in ms.
 * \param bit_polarity  Bit polarity.
 * \param read_index    Nav bit FIFO read index to which the above values
 *                      are synchronized.
 *
 * \return true if outputs are valid, false otherwise.
 */
static bool nav_time_sync_get(nav_time_sync_t *sync, s32 *TOW_ms,
                              s8 *bit_polarity, nav_bit_fifo_index_t *read_index)
{
  bool result = false;
  chMtxLock(&nav_time_sync_mutex);

  if (sync->valid) {
    *TOW_ms = sync->TOW_ms;
    *bit_polarity = sync->bit_polarity;
    *read_index = sync->read_index;
    sync->valid = false;
    result = true;
  }

  Mutex *m = chMtxUnlock();
  assert(m == &nav_time_sync_mutex);
  return result;
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

/** Compress a 32 bit integration value down to 8 bits. */
s8 nav_bit_quantize(s32 bit_integrate)
{
  //  0 through  2^24 - 1 ->  0 = weakest positive bit
  // -1 through -2^24     -> -1 = weakest negative bit

  if (bit_integrate >= 0)
    return bit_integrate / (1 << 24);
  else
    return ((bit_integrate + 1) / (1 << 24)) - 1;
}

/** Initialises a tracking channel.
 * Initialises a tracking channel on the Swift NAP. The start_sample_count
 * must be contrived to be at or close to a PRN edge (PROMPT code phase = 0).
 *
 * \param prn                PRN number - 1 (0-31).
 * \param channel            Tracking channel number on the Swift NAP.
 * \param carrier_freq       Carrier frequency (Doppler) at start of tracking in Hz.
 * \param start_sample_count Sample count on which to start tracking.
 * \param cn0_init           Estimated C/N0 from acquisition
 * \param elevation          Satellite elevation in degrees, or
 *                           TRACKING_ELEVATION_UNKNOWN
 */
void tracking_channel_init(u8 channel, gnss_signal_t sid, float carrier_freq,
                           u32 start_sample_count, float cn0_init, s8 elevation)
{
  tracking_channel_t *chan = &tracking_channel[channel];

  /* Initialize all fields in the channel to 0 */
  memset(chan, 0, sizeof(tracking_channel_t));

  bit_sync_init(&chan->bit_sync, sid);
  nav_bit_fifo_init(&chan->nav_bit_fifo);
  nav_time_sync_init(&chan->nav_time_sync);

  /* Adjust the channel start time as the start_sample_count passed
   * in corresponds to a PROMPT code phase rollover but we want to
   * start the channel on an EARLY code phase rollover.
   */
  /* TODO : change hardcoded sample rate */
  start_sample_count -= 0.5*16;

  /* Setup tracking_channel struct. */
  chan->state = TRACKING_RUNNING;
  chan->sid = sid;
  chan->elevation = elevation;

  /* Initialize TOW_ms and lock_count. */
  tracking_channel_ambiguity_unknown(channel);
  chan->TOW_ms = TOW_INVALID;
  chan->bit_polarity = BIT_POLARITY_UNKNOWN;

  const struct loop_params *l = &loop_params_stage[0];

  /* Note: The only coherent integration interval currently supported
     for first-stage tracking (i.e. loop_params_stage[0].coherent_ms)
     is 1. */
  chan->int_ms = MIN(l->coherent_ms, chan->bit_sync.bit_length);

  /* Calculate code phase rate with carrier aiding. */
  float code_phase_rate = (1 + carrier_freq/GPS_L1_HZ) * GPS_CA_CHIPPING_RATE;

  aided_tl_init(&(chan->tl_state), 1e3 / chan->int_ms,
                code_phase_rate - GPS_CA_CHIPPING_RATE,
                l->code_bw, l->code_zeta, l->code_k,
                l->carr_to_code,
                carrier_freq,
                l->carr_bw, l->carr_zeta, l->carr_k,
                l->carr_fll_aid_gain);

  chan->code_phase_rate_fp = code_phase_rate*NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;
  chan->code_phase_rate_fp_prev = chan->code_phase_rate_fp;
  chan->code_phase_rate = code_phase_rate;
  chan->carrier_freq = carrier_freq;
  chan->carrier_freq_fp = (s32)(carrier_freq * NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ);
  chan->carrier_freq_fp_prev = chan->carrier_freq_fp;
  chan->sample_count = start_sample_count;
  chan->short_cycle = true;

  /* Initialise C/N0 estimator */
  cn0_est_init(&chan->cn0_est, 1e3/chan->int_ms, cn0_init, CN0_EST_LPF_CUTOFF, 1e3/chan->int_ms);

  lock_detect_init(&chan->lock_detect,
                   lock_detect_params.k1, lock_detect_params.k2,
                   lock_detect_params.lp, lock_detect_params.lo);

  /* TODO: Reconfigure alias detection between stages */
  u8 alias_detect_ms = MIN(loop_params_stage[1].coherent_ms,
                           chan->bit_sync.bit_length);
  alias_detect_init(&chan->alias_detect, 500/alias_detect_ms,
                    (alias_detect_ms-1)*1e-3);

  /* Starting carrier phase is set to zero as we don't
   * know the carrier freq well enough to calculate it.
   */
  /* Start with code phase of zero as we have conspired for the
   * channel to be initialised on an EARLY code phase rollover.
   */
  nap_track_code_wr_blocking(channel, sid);
  nap_track_init_wr_blocking(channel, 0, 0, 0);
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
static void tracking_channel_get_corrs(u8 channel)
{
  tracking_channel_t* chan = &tracking_channel[channel];

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
}

/** Force a satellite to drop.
 * This function is used for testing.  It clobbers the code frequency in the
 * loop filter which destroys the correlations.  The satellite is dropped
 * by manage.c which checks the SNR.
 */
void tracking_drop_satellite(gnss_signal_t sid)
{
  for (u8 i=0; i<nap_track_n_channels; i++) {
    if (!sid_is_equal(tracking_channel[i].sid, sid))
      continue;

    tracking_channel[i].tl_state.code_filt.y += 500;
  }
}

/** Determines if C/NO is above the use threshold for a tracking channel.
 * \param channel Tracking channel to use.
 */
bool tracking_channel_cn0_useable(u8 channel)
{
  return (tracking_channel[channel].cn0 > track_cn0_use_thres);
}

/** Update tracking channels after the end of an integration period.
 * Update update_count, sample_count, TOW, run loop filters and update
 * SwiftNAP tracking channel frequencies.
 * \param channel Tracking channel to update.
 */
void tracking_channel_update(u8 channel)
{
  tracking_channel_t* chan = &tracking_channel[channel];

  char buf[SID_STR_LEN_MAX];
  sid_to_string(buf, sizeof(buf), chan->sid);

  switch(chan->state)
  {
    case TRACKING_RUNNING:
    {
      tracking_channel_get_corrs(channel);

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

      /* Latch TOW from nav message if pending */
      s32 pending_TOW_ms;
      s8 pending_bit_polarity;
      nav_bit_fifo_index_t pending_TOW_read_index;
      if (nav_time_sync_get(&chan->nav_time_sync, &pending_TOW_ms,
                            &pending_bit_polarity, &pending_TOW_read_index)) {

        /* Compute time since the pending data was read from the FIFO */
        nav_bit_fifo_index_t fifo_length =
          NAV_BIT_FIFO_INDEX_DIFF(chan->nav_bit_fifo.write_index,
                                  pending_TOW_read_index);
        u32 fifo_time_diff_ms = fifo_length * chan->bit_sync.bit_length;

        /* Add full bit times + fractional bit time to the specified TOW */
        s32 TOW_ms = pending_TOW_ms + fifo_time_diff_ms +
                       chan->nav_bit_TOW_offset_ms;

        if (TOW_ms >= GPS_WEEK_LENGTH_ms)
          TOW_ms -= GPS_WEEK_LENGTH_ms;

        /* Log error if updated TOW does not match current value */
        if ((chan->TOW_ms != TOW_INVALID) && (chan->TOW_ms != TOW_ms)) {
          log_error("%s TOW mismatch: %ld, %lu", buf, chan->TOW_ms, TOW_ms);
        }
        chan->TOW_ms = TOW_ms;
        chan->bit_polarity = pending_bit_polarity;
      }

      u8 int_ms = chan->short_cycle ? 1 : (chan->int_ms-1);
      chan->nav_bit_TOW_offset_ms += int_ms;

      if (chan->TOW_ms != TOW_INVALID) {
        /* Have a valid time of week - increment it. */
        chan->TOW_ms += int_ms;
        if (chan->TOW_ms >= GPS_WEEK_LENGTH_ms)
          chan->TOW_ms -= GPS_WEEK_LENGTH_ms;
        /* TODO: maybe keep track of week number in channel state, or
           derive it from system time */
      }

      if (chan->int_ms > 1) {
        /* If we're doing long integrations, alternate between short and long
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

      /* Update bit sync */
      s32 bit_integrate;
      if (bit_sync_update(&chan->bit_sync, chan->cs[1].I, chan->int_ms,
                          &bit_integrate)) {

        s8 soft_bit = nav_bit_quantize(bit_integrate);

        // write to FIFO
        nav_bit_fifo_element_t element = { .soft_bit = soft_bit };
        if (!nav_bit_fifo_write(&chan->nav_bit_fifo, &element)) {
          log_warn("%s nav bit FIFO overrun", buf);
        }

        // clear nav bit TOW offset
        chan->nav_bit_TOW_offset_ms = 0;
      }

      /* Correlations should already be in chan->cs thanks to
       * tracking_channel_get_corrs. */
      corr_t* cs = chan->cs;

      /* Update C/N0 estimate */
      chan->cn0 = cn0_est(&chan->cn0_est, cs[1].I/chan->int_ms, cs[1].Q/chan->int_ms);
      if (chan->cn0 > track_cn0_drop_thres)
        chan->cn0_above_drop_thres_count = chan->update_count;

      if (chan->cn0 < track_cn0_use_thres) {
        /* SNR has dropped below threshold, indicate that the carrier phase
         * ambiguity is now unknown as cycle slips are likely. */
        tracking_channel_ambiguity_unknown(channel);
        /* Update the latest time we were below the threshold. */
        chan->cn0_below_threshold_count = chan->update_count;
      }

      /* Update PLL lock detector */
      bool last_outp = chan->lock_detect.outp;
      lock_detect_update(&chan->lock_detect, cs[1].I, cs[1].Q, chan->int_ms);
      if (chan->lock_detect.outo)
        chan->ld_opti_locked_count = chan->update_count;

      /* Reset carrier phase ambiguity if there's doubt as to our phase lock */
      if (last_outp && !chan->lock_detect.outp) {
        if (chan->stage > 0) {
          log_info("%s PLL stress", buf);
        }
        tracking_channel_ambiguity_unknown(channel);
      }

      /* Run the loop filters. */

      /* TODO: Make this more elegant. */
      correlation_t cs2[3];
      for (u32 i = 0; i < 3; i++) {
        cs2[i].I = cs[2-i].I;
        cs2[i].Q = cs[2-i].Q;
      }

      /* Output I/Q correlations using SBP if enabled for this channel */
      if (chan->output_iq && (chan->int_ms > 1)) {
        msg_tracking_iq_t msg = {
          .channel = channel,
        };
        msg.sid = sid_to_sbp(chan->sid);
        for (u32 i = 0; i < 3; i++) {
          msg.corrs[i].I = cs[i].I;
          msg.corrs[i].Q = cs[i].Q;
        }
        sbp_send_msg(SBP_MSG_TRACKING_IQ, sizeof(msg), (u8*)&msg);
      }

      aided_tl_update(&(chan->tl_state), cs2);
      chan->carrier_freq = chan->tl_state.carr_freq;
      chan->code_phase_rate = chan->tl_state.code_freq + GPS_CA_CHIPPING_RATE;

      chan->code_phase_rate_fp_prev = chan->code_phase_rate_fp;
      chan->code_phase_rate_fp = chan->code_phase_rate
        * NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;

      chan->carrier_freq_fp = chan->carrier_freq
        * NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ;

      /* Attempt alias detection if we have pessimistic phase lock detect, OR
         (optimistic phase lock detect AND are in second-stage tracking) */
      if (use_alias_detection &&
          (chan->lock_detect.outp ||
           (chan->lock_detect.outo && chan->stage > 0))) {
        s32 I = (cs[1].I - chan->alias_detect.first_I) / (chan->int_ms - 1);
        s32 Q = (cs[1].Q - chan->alias_detect.first_Q) / (chan->int_ms - 1);
        float err = alias_detect_second(&chan->alias_detect, I, Q);
        if (fabs(err) > (250 / chan->int_ms)) {
          if (chan->lock_detect.outp) {
            log_warn("False phase lock detected on %s: err=%f", buf, err);
          }

          tracking_channel_ambiguity_unknown(channel);
          /* Indicate that a mode change has occurred. */
          chan->mode_change_count = chan->update_count;

          chan->tl_state.carr_freq += err;
          chan->tl_state.carr_filt.y = chan->tl_state.carr_freq;
        }
      }

      /* Consider moving from stage 0 (1 ms integration) to stage 1 (longer). */
      if ((chan->stage == 0) &&
          /* Must have (at least optimistic) phase lock */
          (chan->lock_detect.outo) &&
          /* Must have nav bit sync, and be correctly aligned */
          (chan->bit_sync.bit_phase == chan->bit_sync.bit_phase_ref)) {
        log_info("%s synced @ %u ms, %.1f dBHz",
                 buf, (unsigned int)chan->update_count, chan->cn0);
        chan->stage = 1;
        const struct loop_params *l = &loop_params_stage[1];
        chan->int_ms = MIN(l->coherent_ms, chan->bit_sync.bit_length);
        chan->short_cycle = true;

        cn0_est_init(&chan->cn0_est, 1e3 / chan->int_ms, chan->cn0,
                     CN0_EST_LPF_CUTOFF, 1e3 / chan->int_ms);

        /* Recalculate filter coefficients */
        aided_tl_retune(&chan->tl_state, 1e3 / chan->int_ms,
                        l->code_bw, l->code_zeta, l->code_k,
                        l->carr_to_code,
                        l->carr_bw, l->carr_zeta, l->carr_k,
                        l->carr_fll_aid_gain);

        lock_detect_reinit(&chan->lock_detect,
                           lock_detect_params.k1 * chan->int_ms,
                           lock_detect_params.k2,
                           /* TODO: Should also adjust lp and lo? */
                           lock_detect_params.lp, lock_detect_params.lo);

        /* Indicate that a mode change has occurred. */
        chan->mode_change_count = chan->update_count;
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
      /* TODO: Why do we sometimes have to disable the channel in the
               NAP repeatedly? */
      tracking_channel_disable(channel);
      break;
    default:
      log_error("CH%d (%s) invalid state %d", channel, buf, chan->state);
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
 * invalidates the half cycle ambiguity, which must be resolved again by the navigation
 *  message processing. Should be called if a cycle slip is suspected.
 *
 * \param channel Tracking channel number to mark phase-ambiguous.
 */
void tracking_channel_ambiguity_unknown(u8 channel)
{
  gnss_signal_t sid = tracking_channel[channel].sid;

  tracking_channel[channel].bit_polarity = BIT_POLARITY_UNKNOWN;
  tracking_channel[channel].lock_counter =
      ++tracking_lock_counters[sid_to_global_index(sid)];
}

/** Update channel measurement for a tracking channel.
 * \param channel Tracking channel to update measurement from.
 * \param meas Pointer to channel_measurement_t where measurement will be put.
 */
void tracking_update_measurement(u8 channel, channel_measurement_t *meas)
{
  tracking_channel_t* chan = &tracking_channel[channel];

  /* Update our channel measurement. */
  meas->sid = chan->sid;
  meas->code_phase_chips = (double)chan->code_phase_early / NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP;
  meas->code_phase_rate = chan->code_phase_rate;
  meas->carrier_phase = chan->carrier_phase / (double)(1<<24);
  meas->carrier_freq = chan->carrier_freq;
  meas->time_of_week_ms = chan->TOW_ms;
  meas->receiver_time = (double)chan->sample_count / SAMPLE_FREQ;
  meas->snr = chan->cn0;
  if (chan->bit_polarity == BIT_POLARITY_INVERTED) {
    meas->carrier_phase += 0.5;
  }
  meas->lock_counter = chan->lock_counter;
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
        states[i].sid.code = 0;
        states[i].sid.sat = 0;
        states[i].cn0 = -1;
      }
    }

  } else {

    for (u8 i=0; i<nap_track_n_channels; i++) {
      states[i].state = tracking_channel[i].state;
      states[i].sid = sid_to_sbp(tracking_channel[i].sid);
      if (tracking_channel[i].state == TRACKING_RUNNING)
        states[i].cn0 = tracking_channel[i].cn0;
      else
        states[i].cn0 = -1;
    }

  }

  sbp_send_msg(SBP_MSG_TRACKING_STATE, sizeof(states), (u8*)states);

}

/** Parse a string describing the tracking loop filter parameters into
    the loop_params_stage structs. */
static bool parse_loop_params(struct setting *s, const char *val)
{
  /** The string contains loop parameters for either one or two
      stages.  If the second is omitted, we'll use the same parameters
      as the first stage.*/

  struct loop_params loop_params_parse[2];

  const char *str = val;
  for (int stage = 0; stage < 2; stage++) {
    struct loop_params *l = &loop_params_parse[stage];

    int n_chars_read = 0;
    unsigned int tmp; /* newlib's sscanf doesn't support hh size modifier */

    if (sscanf(str, "( %u ms , ( %f , %f , %f , %f ) , ( %f , %f , %f , %f ) ) , %n",
               &tmp,
               &l->code_bw, &l->code_zeta, &l->code_k, &l->carr_to_code,
               &l->carr_bw, &l->carr_zeta, &l->carr_k, &l->carr_fll_aid_gain,
               &n_chars_read) < 9) {
      log_error("Ill-formatted tracking loop param string.");
      return false;
    }
    l->coherent_ms = tmp;
    /* If string omits second-stage parameters, then after the first
       stage has been parsed, n_chars_read == 0 because of missing
       comma and we'll parse the string again into loop_params_parse[1]. */
    str += n_chars_read;

    if ((l->coherent_ms == 0)
        || ((20 % l->coherent_ms) != 0) /* i.e. not 1, 2, 4, 5, 10 or 20 */
        || (stage == 0 && l->coherent_ms != 1)) {
      log_error("Invalid coherent integration length.");
      return false;
    }
  }
  /* Successfully parsed both stages.  Save to memory. */
  strncpy(s->addr, val, s->len);
  memcpy(loop_params_stage, loop_params_parse, sizeof(loop_params_stage));
  return true;
}

/** Parse a string describing the tracking loop phase lock detector
    parameters into the lock_detect_params structs. */
static bool parse_lock_detect_params(struct setting *s, const char *val)
{
  struct lock_detect_params p;

  if (sscanf(val, "%f , %f , %" SCNu16 " , %" SCNu16,
             &p.k1, &p.k2, &p.lp, &p.lo) < 4) {
      log_error("Ill-formatted lock detect param string.");
      return false;
  }
  /* Successfully parsed.  Save to memory. */
  strncpy(s->addr, val, s->len);
  memcpy(&lock_detect_params, &p, sizeof(lock_detect_params));
  return true;
}

bool track_iq_output_notify(struct setting *s, const char *val)
{
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    for (int i = 0; i < NAP_MAX_N_TRACK_CHANNELS; i++) {
      tracking_channel[i].output_iq = (iq_output_mask & (1 << i)) != 0;
    }
    return true;
  }
  return false;
}


/** Set up tracking subsystem
 */
void tracking_setup()
{
  SETTING_NOTIFY("track", "iq_output_mask", iq_output_mask, TYPE_INT,
                 track_iq_output_notify);
  SETTING_NOTIFY("track", "loop_params", loop_params_string,
                 TYPE_STRING, parse_loop_params);
  SETTING_NOTIFY("track", "lock_detect_params", lock_detect_params_string,
                 TYPE_STRING, parse_lock_detect_params);
  SETTING("track", "cn0_use", track_cn0_use_thres, TYPE_FLOAT);
  SETTING("track", "cn0_drop", track_cn0_drop_thres, TYPE_FLOAT);
  SETTING("track", "alias_detect", use_alias_detection, TYPE_BOOL);

  for (u32 i=0; i < PLATFORM_SIGNAL_COUNT; i++) {
    tracking_lock_counters[i] = random_int();
  }
}

/** Read the next pending nav bit for a tracking channel.
 *
 * \note This function should should be called from the same thread as
 * tracking_channel_time_sync().
 *
 * \param channel     Tracking channel to use.
 * \param soft_bit    Output soft nav bit value.
 *
 * \return true if *soft_bit is valid, false otherwise.
 */
bool tracking_channel_nav_bit_get(u8 channel, s8 *soft_bit)
{
  tracking_channel_t *chan = &tracking_channel[channel];
  nav_bit_fifo_element_t element;
  if (nav_bit_fifo_read(&chan->nav_bit_fifo, &element)) {
    *soft_bit = element.soft_bit;
    return true;
  }
  return false;
}

/** Propagate decoded time of week and bit polarity back to a tracking channel.
 *
 * \note This function should be called from the same thread as
 * tracking_channel_nav_bit_get().
 * \note It is assumed that the specified data is synchronized with the most
 * recent nav bit read from the FIFO using tracking_channel_nav_bit_get().
 *
 * \param channel       Tracking channel to use.
 * \param TOW_ms        Time of week in milliseconds.
 * \param bit_polarity  Bit polarity.
 *
 * \return true if data was enqueued successfully, false otherwise.
 */
bool tracking_channel_time_sync(u8 channel, s32 TOW_ms, s8 bit_polarity)
{
  assert(TOW_ms >= 0);
  assert(TOW_ms < GPS_WEEK_LENGTH_ms);
  assert((bit_polarity == BIT_POLARITY_NORMAL) ||
         (bit_polarity == BIT_POLARITY_INVERTED));

  tracking_channel_t *chan = &tracking_channel[channel];
  nav_bit_fifo_index_t read_index = chan->nav_bit_fifo.read_index;
  return nav_time_sync_set(&chan->nav_time_sync,
                           TOW_ms, bit_polarity, read_index);
}

/** \} */

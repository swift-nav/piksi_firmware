/*
 * Copyright (C) 2011-2014,2016 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "nap_conf.h"
#include "nap_common.h"
#include "track_channel.h"
#include "track.h"
#include "main.h"

#include "libswiftnav/prns.h"
#include "libswiftnav/constants.h"

#include <string.h>

/** \addtogroup nap
 * \{ */

/** \defgroup track_channel Track Channel
 * Interface to the SwiftNAP track channels.
 * \{ */

/* NAP track channel parameters.
 * TODO : get rid of some of these INIT reg specific defines by just writing
 * whole phase through init register? Tracking channel init registers don't
 * get written very often so it shouldn't increase SPI link budget much.
 */
#define NAP_TRACK_CARRIER_FREQ_WIDTH              24
#define NAP_TRACK_CODE_PHASE_WIDTH                29
#define NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH     32

#define NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ       \
  ((1 << NAP_TRACK_CARRIER_FREQ_WIDTH) / (double)SAMPLE_FREQ)

#define NAP_TRACK_NOMINAL_CODE_PHASE_RATE         \
  (1 << (NAP_TRACK_CODE_PHASE_WIDTH - 1))

#define NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ    \
  (NAP_TRACK_NOMINAL_CODE_PHASE_RATE / 1.023e6)

#define NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP       \
  ((u64)1 << NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH)

static struct nap_ch_state {
  u32 code_pinc;
  u32 code_pinc_prev;
  s32 carr_pinc;
  s32 carr_pinc_prev;
  u32 code_phase;
  s64 carrier_phase;
  u32 count_snapshot;
} nap_ch_state[NAP_MAX_N_TRACK_CHANNELS];

/** \addtogroup nap
 * \{ */

/** Write to a NAP track channel's INIT register.
 * Sets PRN (deprecated), initial carrier phase, and initial code phase of a
 * NAP track channel. The tracking channel will start correlating with these
 * parameters at the falling edge of the next NAP internal timing strobe.
 * Also write CA code to track channel's code ram.
 *
 * \note The track channel's UPDATE register, which sets the carrier and
 *       code phase rates, must also be written to before the internal timing
 *       strobe goes low.
 *
 * \param channel       NAP track channel whose INIT register to write.
 * \param prn           CA code PRN (0-31) to track. (deprecated)
 * \param carrier_phase Initial code phase.
 * \param code_phase    Initial carrier phase.
 * \param chips_to_correlate How many chips to correlate (unused)
 */
void nap_track_init(u8 channel, gnss_signal_t sid, u32 ref_timing_count,
                    float carrier_freq, float code_phase, u32 chips_to_correlate)
{
  struct nap_ch_state *s = &nap_ch_state[channel];
  memset(s, 0, sizeof(*s));

  (void) chips_to_correlate;

  u32 track_count = nap_timing_count() + 20000;
  float cp = propagate_code_phase(code_phase, carrier_freq,
                                  track_count - ref_timing_count, sid.code);

  /* Contrive for the timing strobe to occur at or close to a
   * PRN edge (code phase = 0) */
  track_count += (SAMPLE_FREQ/GPS_CA_CHIPPING_RATE) * (1023.0-cp) *
                 (1.0 + carrier_freq / GPS_L1_HZ);

  nap_track_code_wr_blocking(channel, sid);
  nap_track_init_wr_blocking(channel, 0, 0, 0);

  double cp_rate = (1 + carrier_freq/GPS_L1_HZ) * GPS_CA_CHIPPING_RATE;
  nap_track_update(channel, carrier_freq, cp_rate, 0, 0);

  /* Schedule the timing strobe for start_sample_count. */
  track_count -= SAMPLE_FREQ / (2*GPS_CA_CHIPPING_RATE);
  
  s->count_snapshot = track_count;
  s->carrier_phase = -s->carr_pinc;
  s->carr_pinc_prev = s->carr_pinc;
  s->code_pinc_prev = s->code_pinc;

  COMPILER_BARRIER();

  nap_timing_strobe(track_count);
  nap_timing_strobe_wait(100);
}


/** Write to a NAP track channel's UPDATE register.
 * Write new carrier frequency and code phase rate to a NAP track channel's
 * UPDATE register, which will be used to accumulate the channel's carrier and
 * code phases during the next correlation period.
 *
 * \note This must be called in addition to nap_track_init_wr_blocking when a
 *       new track channel is being set up, before the NAP's internal timing
 *       strobe goes low.
 *
 * \note If two track channel IRQ's occur without a write to the tracking
 *       channel's UPDATE register between them, the error bit for the track
 *       channel in the NAP error register will go high.
 *
 * \param channel         NAP track channel whose UPDATE register to write.
 * \param carrier_freq    Next correlation period's carrier frequency.
 * \param code_phase_rate Next correlation period's code phase rate.
 * \param chips_to_correlate How many chips to correlate over.
 * \param corr_spacing    Spacing between correlator's EPL replicas.
 */
void nap_track_update(u8 channel, double carrier_freq,
                      double code_phase_rate, u32 chips_to_correlate,
                      u8 corr_spacing)
{
  u8 rollover_count;
  struct nap_ch_state *s = &nap_ch_state[channel];
  s->carr_pinc_prev = s->carr_pinc;
  s->carr_pinc = (s32)(carrier_freq * NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ);
  s->code_pinc_prev = s->code_pinc;
  s->code_pinc = code_phase_rate * NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;

  rollover_count = (u8) (chips_to_correlate / GPS_L1CA_CHIPS_NUM);
  if (rollover_count) {
    rollover_count -= 1;
  }
  nap_track_update_wr_blocking(channel, s->carr_pinc, s->code_pinc,
                               rollover_count, corr_spacing);
}

/** Read data from a NAP track channel's CORR register.
 *
 * \param channel      NAP track channel whose CORR register to read.
 * \param sample_count Number of sample clock cycles in correlation period.
 * \param corrs        Array of E,P,L correlations from correlation period.
 */
void nap_track_read_results(u8 channel,
                            u32* count_snapshot, corr_t corrs[],
                            double *code_phase_early,
                            double *carrier_phase)
{
  struct nap_ch_state *s = &nap_ch_state[channel];
  u32 sample_count;

  nap_track_corr_rd_blocking(channel, &sample_count, corrs);
  
  s->count_snapshot += sample_count;
  s->code_phase += (u64)sample_count * s->code_pinc_prev;
  s->carrier_phase += (s64)sample_count * s->carr_pinc_prev;
  s->carr_pinc_prev = s->carr_pinc;
  s->code_pinc_prev = s->code_pinc_prev;

  *count_snapshot = s->count_snapshot;
  *code_phase_early = (double)s->code_phase / NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP;
  *carrier_phase = (double)s->carrier_phase / (1 << NAP_TRACK_CARRIER_FREQ_WIDTH);
}

void nap_track_disable(u8 channel)
{
  nap_track_update_wr_blocking(channel, 0, 0, 0, 0);
}

/** \} */

/** \} */


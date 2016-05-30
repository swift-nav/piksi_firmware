/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Gareth McMullin <gareth@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "board.h"
#include "nap/nap_common.h"
#include "nap/track_channel.h"
#include "track.h"
#include "main.h"

#include <ch.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>
#include <libswiftnav/prns.h>   /* to expose sid_to_init_g1() declaration */

#include <assert.h>
#include <string.h>

#define TRACK_SAMPLE_FREQ (SAMPLE_FREQ / 4)

/* NAP track channel parameters. */
#define NAP_TRACK_CARRIER_FREQ_WIDTH              32
#define NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH     32

#define NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ       \
  (((u64)1 << NAP_TRACK_CARRIER_FREQ_WIDTH) / (double)TRACK_SAMPLE_FREQ)

#define NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP       \
  ((u64)1 << NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH)

#define NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ    \
  (NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP / TRACK_SAMPLE_FREQ)

#define SPACING_HALF_CHIP         ((u16)(TRACK_SAMPLE_FREQ / GPS_CA_CHIPPING_RATE) / 2)

BSEMAPHORE_DECL(timing_strobe_sem, TRUE);

static struct {
  u32 code_phase;
  u32 len;
} nap_ch_state[NAP_MAX_N_TRACK_CHANNELS];

/** Compute the correlation length in the units of sampling frequency samples.
 * \param chips_to_correlate The number of chips to correlate over.
 * \param cp_start Initial code phase in NAP units.
 * \param cp_rate Code phase rate.
 * \return The correlation length in NAP units
 */
static u32 calc_length_samples(u32 chips_to_correlate, s32 cp_start, u32 cp_rate)
{
  u64 cp_units = (1ULL << NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH) *
                 chips_to_correlate - cp_start;
  u32 samples = cp_units / cp_rate;
  return samples;
}

/** Looks-up RF frontend channel for the given signal ID.
 * \param sid Signal ID.
 * \return RF front-end channel number.
 */
u8 sid_to_rf_frontend_channel(gnss_signal_t sid)
{
  u8 ret = ~0;
  switch (sid.code) {
  case CODE_GPS_L1CA:
  case CODE_SBAS_L1CA:
    ret = NAP_RF_FRONTEND_CHANNEL_1;
    break;
  case CODE_GPS_L2CM:
    ret = NAP_RF_FRONTEND_CHANNEL_4;
    break;
  default:
    assert(0);
    break;
  }
  return ret;
}

/** Looks-up NAP constellation and band code for the given signal ID.
 * \param sid Signal ID.
 * \return NAP constallation and band code.
 */
u8 sid_to_nap_code(gnss_signal_t sid)
{
  u8 ret = ~0;
  switch (sid.code) {
  case CODE_GPS_L1CA:
  case CODE_SBAS_L1CA:
    ret = NAP_CODE_GPS_L1CA_SBAS_L1CA;
    break;
  case CODE_GPS_L2CM:
    ret = NAP_CODE_GPS_L2CM;
    break;
  default:
    assert(0);
    break;
  }
  return ret;
}

void nap_track_init(u8 channel, gnss_signal_t sid, u32 ref_timing_count,
                   float carrier_freq, float code_phase, u32 chips_to_correlate)
{
  u16 control;
  u32 now = NAP->TIMING_COUNT;
  u32 track_count = now + 200000;
  double cp = propagate_code_phase(code_phase, carrier_freq,
                                  track_count - ref_timing_count, sid.code);

  /* Contrive for the timing strobe to occur at or close to a
   * PRN edge (code phase = 0) */
  track_count += (SAMPLE_FREQ / code_to_chip_rate(sid.code)) *
                 (code_to_chip_count(sid.code) - cp) *
                 (1.0 + carrier_freq / code_to_carr_freq(sid.code));

  u8 prn = sid.sat - 1;
  /* PRN code */
  control = (prn << NAP_TRK_CONTROL_SAT_Pos) & NAP_TRK_CONTROL_SAT_Msk;
  /* RF frontend channel */
  control |= (sid_to_rf_frontend_channel(sid) << NAP_TRK_CONTROL_RF_FE_Pos) &
             NAP_TRK_CONTROL_RF_FE_Msk;
  /* Constellation and band for tracking */
  control |= (sid_to_nap_code(sid) << NAP_TRK_CONTROL_CODE_Pos) &
             NAP_TRK_CONTROL_CODE_Msk;
  /* We are not utilizing multiple signals within one RF channel at the moment.
     Therefore, RF_FE_CH is 0 and below statement in a NOP. */
  control |= (0 << NAP_TRK_CONTROL_RF_FE_CH_Pos) & NAP_TRK_CONTROL_RF_FE_CH_Msk;

  NAP->TRK_CH[channel].CONTROL = control;
  /* We always start at zero code phase */
  NAP->TRK_CH[channel].CODE_INIT_INT = 0;
  NAP->TRK_CH[channel].CODE_INIT_FRAC = 0;
  NAP->TRK_CH[channel].CODE_INIT_G1 = sid_to_init_g1(sid);
  NAP->TRK_CH[channel].CODE_INIT_G2 = 0x3ff;

  NAP->TRK_CH[channel].SPACING = (SPACING_HALF_CHIP << 16) | SPACING_HALF_CHIP;

  u32 cp_rate = (1.0 + carrier_freq / code_to_carr_freq(sid.code)) *
                code_to_chip_rate(sid.code) *
                NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;

  NAP->TRK_CH[channel].CARR_PINC = -carrier_freq *
                                   NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ;
  NAP->TRK_CH[channel].CODE_PINC = cp_rate;

  u32 length = calc_length_samples(chips_to_correlate, 0, cp_rate) + 1;
  nap_ch_state[channel].len = NAP->TRK_CH[channel].LENGTH = length;
  nap_ch_state[channel].code_phase = (NAP->TRK_CH[channel].LENGTH) * cp_rate;
  NAP->TRK_CONTROL |= (1 << channel); /* Set to start on the timing strobe */

  COMPILER_BARRIER();

  NAP->TRK_TIMING_COMPARE = track_count - SAMPLE_FREQ / GPS_CA_CHIPPING_RATE;
  chThdSleep(CH_CFG_ST_FREQUENCY * ceil((float)(track_count - now)/SAMPLE_FREQ));
}

void nap_track_update(u8 channel, double carrier_freq,
                      double code_phase_rate, u32 chips_to_correlate,
                      u8 corr_spacing)
{
  (void)corr_spacing; /* This is always written as 0 now... */

  u32 cp_rate_fp = code_phase_rate * NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;
  NAP->TRK_CH[channel].CARR_PINC = -carrier_freq *
                                   NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ;
  NAP->TRK_CH[channel].CODE_PINC = cp_rate_fp;
  u32 length = calc_length_samples(chips_to_correlate,
                                   nap_ch_state[channel].code_phase,
                                   cp_rate_fp);
  NAP->TRK_CH[channel].LENGTH = length;

  volatile u16 cp_int = NAP->TRK_CH[channel].CODE_PHASE_INT;
  if ((cp_int != 0x3fe)) /* Sanity check, we should be just before rollover */
    asm("nop");
}

void nap_track_read_results(u8 channel,
                            u32* count_snapshot, corr_t corrs[],
                            double *code_phase_early,
                            double *carrier_phase)
{
  corr_t lc[5];
  if (NAP->TRK_CH[channel].STATUS & 0x3F)
    log_warn("Track correlator overflow 0x%08X on channel %d",
              NAP->TRK_CH[channel].STATUS, channel);
  for (u8 i = 0; i < 5; i++) {
    lc[i].I = NAP->TRK_CH[channel].CORR[i].I >> 8;
    lc[i].Q = NAP->TRK_CH[channel].CORR[i].Q >> 8;
  }
  *count_snapshot = NAP->TRK_CH[channel].START_SNAPSHOT;
  u64 nap_code_phase = ((u64)NAP->TRK_CH[channel].CODE_PHASE_INT << 32) |
                             NAP->TRK_CH[channel].CODE_PHASE_FRAC;
  s64 nap_carr_phase = ((s64)NAP->TRK_CH[channel].CARR_PHASE_INT << 32) |
                             NAP->TRK_CH[channel].CARR_PHASE_FRAC;
  *code_phase_early = (double)nap_code_phase / NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP;
  *carrier_phase = (double)-nap_carr_phase / (1ull << 32);
  memcpy(corrs, &lc[1], sizeof(corr_t)*3);
  if (nap_ch_state[channel].code_phase != NAP->TRK_CH[channel].CODE_PHASE_FRAC)
    asm("nop");
  nap_ch_state[channel].code_phase +=
    NAP->TRK_CH[channel].LENGTH * NAP->TRK_CH[channel].CODE_PINC;
  nap_ch_state[channel].len = NAP->TRK_CH[channel].LENGTH;
  if (!(NAP->STATUS & 1))
    asm("bkpt");
}

void nap_track_disable(u8 channel)
{
  NAP->TRK_CONTROL &= ~(1 << channel); /* Set to start on the timing strobe */
}


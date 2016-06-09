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

static u32 calc_length_samples(u8 codes, s32 cp_start, u32 cp_rate)
{
  u16 chips = codes * 1023;
  u64 cp_units = (1ULL << NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH) * chips
                 - cp_start;
  u32 samples = cp_units / cp_rate;
  return samples;
}

void nap_track_init(u8 channel, gnss_signal_t sid, u32 ref_timing_count,
                   float carrier_freq, float code_phase)
{
  u32 now = NAP->TIMING_COUNT;
  u32 track_count = now + 200000;
  double cp = propagate_code_phase(code_phase, carrier_freq,
                                  track_count - ref_timing_count);

  /* Contrive for the timing strobe to occur at or close to a
   * PRN edge (code phase = 0) */
  track_count += (SAMPLE_FREQ/GPS_CA_CHIPPING_RATE) * (1023.0-cp) *
                 (1.0 + carrier_freq / GPS_L1_HZ);

  u8 prn = sid.sat - 1;
  NAP->TRK_CH[channel].CONTROL = prn << NAP_TRK_CONTROL_SAT_Pos;
  /* We always start at zero code phase */
  NAP->TRK_CH[channel].CODE_INIT_INT = 0;
  NAP->TRK_CH[channel].CODE_INIT_FRAC = 0;
  NAP->TRK_CH[channel].CODE_INIT_G1 = 0x3ff;
  NAP->TRK_CH[channel].CODE_INIT_G2 = 0x3ff;

  NAP->TRK_CH[channel].SPACING = (SPACING_HALF_CHIP << 16) | SPACING_HALF_CHIP;

  u32 cp_rate = (1.0 + carrier_freq/GPS_L1_HZ) * GPS_CA_CHIPPING_RATE *
                NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;

  NAP->TRK_CH[channel].CARR_PINC = -carrier_freq *
                                   NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ;
  NAP->TRK_CH[channel].CODE_PINC = cp_rate;
  nap_ch_state[channel].len = NAP->TRK_CH[channel].LENGTH = calc_length_samples(1, 0, cp_rate)+1;
  nap_ch_state[channel].code_phase = (NAP->TRK_CH[channel].LENGTH) * cp_rate;
  NAP->TRK_CONTROL |= (1 << channel); /* Set to start on the timing strobe */

  COMPILER_BARRIER();

  NAP->TRK_TIMING_COMPARE = track_count - SAMPLE_FREQ / GPS_CA_CHIPPING_RATE;
  chThdSleep(CH_CFG_ST_FREQUENCY * ceil((float)(track_count - now)/SAMPLE_FREQ));
}

void nap_track_update(u8 channel, double carrier_freq,
                      double code_phase_rate, u8 rollover_count,
                      u8 corr_spacing)
{
  (void)corr_spacing; /* This is always written as 0 now... */

  u32 cp_rate_fp = code_phase_rate * NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;
  NAP->TRK_CH[channel].CARR_PINC = -carrier_freq *
                                   NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ;
  NAP->TRK_CH[channel].CODE_PINC = cp_rate_fp;
  NAP->TRK_CH[channel].LENGTH = calc_length_samples(rollover_count + 1,
                                    nap_ch_state[channel].code_phase,
                                    cp_rate_fp);

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
  if (NAP->TRK_CH[channel].STATUS & NAP_TRK_STATUS_CORR_OVF_Msk)
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
}

void nap_track_disable(u8 channel)
{
  NAP->TRK_CONTROL &= ~(1 << channel); /* Set to start on the timing strobe */
}


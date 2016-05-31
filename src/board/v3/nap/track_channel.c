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

#include "nap_constants.h"
#include "nap_hw.h"
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


#define TIMING_COMPARE_DELTA_MIN (  1e-3 * TRACK_SAMPLE_FREQ) /*   1ms */
#define TIMING_COMPARE_DELTA_MAX (100e-3 * TRACK_SAMPLE_FREQ) /* 100ms */

/* NAP track channel parameters. */
#define NAP_TRACK_CARRIER_FREQ_WIDTH              32
#define NAP_TRACK_CARRIER_PHASE_FRACTIONAL_WIDTH  32
#define NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH     32

#define NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ       \
  (((u64)1 << NAP_TRACK_CARRIER_FREQ_WIDTH) / (double)TRACK_SAMPLE_FREQ)

#define NAP_TRACK_CARRIER_PHASE_UNITS_PER_CYCLE   \
  ((u64)1 << NAP_TRACK_CARRIER_PHASE_FRACTIONAL_WIDTH)

#define NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ    \
  (NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP / (double)TRACK_SAMPLE_FREQ)

#define NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP       \
  ((u64)1 << NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH)

#define SPACING_HALF_CHIP ((u16)(TRACK_SAMPLE_FREQ / GPS_CA_CHIPPING_RATE) / 2)

static struct nap_ch_state {
  u32 code_phase;   /**< Fractional part of code phase. */
} nap_ch_state[NAP_MAX_N_TRACK_CHANNELS];

static u32 calc_length_samples(u8 codes, u32 cp_start_frac_units,
                               u32 cp_rate_units)
{
  u64 cp_end_units = codes * 1023 * NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP;
  /* cp_start_frac_units is reinterpreted as a signed value. This works
   * because NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH is equal to 32 */
  u64 cp_units = cp_end_units - (s32)cp_start_frac_units;
  u32 samples = cp_units / cp_rate_units;
  return samples;
}

void nap_track_init(u8 channel, gnss_signal_t sid, u32 ref_timing_count,
                    float carrier_freq, float code_phase)
{
  assert(sid.code == CODE_GPS_L1CA);

  nap_trk_regs_t *t = &NAP->TRK_CH[channel];
  struct nap_ch_state *s = &nap_ch_state[channel];
  memset(s, 0, sizeof(*s));

  u8 prn = sid.sat - GPS_FIRST_PRN;
  t->CONTROL = prn << NAP_TRK_CONTROL_SAT_Pos;

  /* We always start at zero code phase */
  t->CODE_INIT_INT = 0;
  t->CODE_INIT_FRAC = 0;
  t->CODE_INIT_G1 = 0x3ff;
  t->CODE_INIT_G2 = 0x3ff;

  t->SPACING = (SPACING_HALF_CHIP << NAP_TRK_SPACING_OUTER_Pos) |
               (SPACING_HALF_CHIP << NAP_TRK_SPACING_INNER_Pos);

  double cp_rate = (1.0 + carrier_freq / GPS_L1_HZ) * GPS_CA_CHIPPING_RATE;
  nap_track_update(channel, carrier_freq, cp_rate, 0, 0);

  /* First integration is one sample short */
  t->LENGTH += 1;

  s->code_phase += t->LENGTH * t->CODE_PINC;

  /* Set to start on the timing strobe */
  NAP->TRK_CONTROL |= (1 << channel);

  COMPILER_BARRIER();

  /* Set up timing compare */
  u32 tc_req;
  while (1) {
    chSysLock();
    tc_req = NAP->TIMING_COUNT + TIMING_COMPARE_DELTA_MIN;
    double cp = propagate_code_phase(code_phase, carrier_freq,
                                     tc_req - ref_timing_count);
    /* Contrive for the timing strobe to occur at or close to a
     * PRN edge (code phase = 0) */
    tc_req += (NAP_FRONTEND_SAMPLE_RATE_Hz / GPS_CA_CHIPPING_RATE) * (1023.0 - cp) *
              (1.0 + carrier_freq / GPS_L1_HZ);

    NAP->TRK_TIMING_COMPARE = tc_req;
    chSysUnlock();

    if (tc_req - NAP->TRK_TIMING_SNAPSHOT <= (u32)TIMING_COMPARE_DELTA_MAX) {
      break;
    }
  }

  /* Sleep until compare match */
  s32 tc_delta;
  while ((tc_delta = tc_req - NAP->TIMING_COUNT) >= 0) {
    systime_t sleep_time = ceil(CH_CFG_ST_FREQUENCY * tc_delta / NAP_FRONTEND_SAMPLE_RATE_Hz);
    /* The next system tick will always occur less than the nominal tick period
     * in the future, so sleep for an extra tick. */
    chThdSleep(1 + sleep_time);
  }
}

void nap_track_update(u8 channel, double carrier_freq,
                      double code_phase_rate, u8 rollover_count,
                      u8 corr_spacing)
{
  (void)corr_spacing; /* This is always written as 0 now... */

  nap_trk_regs_t *t = &NAP->TRK_CH[channel];
  struct nap_ch_state *s = &nap_ch_state[channel];

  u32 cp_rate_units = code_phase_rate * NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;
  t->CARR_PINC = -carrier_freq * NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ;
  t->CODE_PINC = cp_rate_units;
  t->LENGTH = calc_length_samples(rollover_count + 1,
                                  s->code_phase, cp_rate_units);
}

void nap_track_read_results(u8 channel,
                            u32* count_snapshot, corr_t corrs[],
                            double *code_phase_early,
                            double *carrier_phase)
{
  nap_trk_regs_t *t = &NAP->TRK_CH[channel];
  struct nap_ch_state *s = &nap_ch_state[channel];

  u32 ovf = (t->STATUS & NAP_TRK_STATUS_OVF_Msk) >> NAP_TRK_STATUS_OVF_Pos;
  if (ovf) {
    log_warn("Track correlator overflow 0x%04X on channel %d", ovf, channel);
  }

  corr_t lc[5];
  for (u8 i = 0; i < 5; i++) {
    lc[i].I = t->CORR[i].I >> 8;
    lc[i].Q = t->CORR[i].Q >> 8;
  }
  memcpy(corrs, &lc[1], sizeof(corr_t)*3);

  u64 nap_code_phase = ((u64)t->CODE_PHASE_INT << 32) |
                             t->CODE_PHASE_FRAC;
  s64 nap_carr_phase = ((s64)t->CARR_PHASE_INT << 32) |
                             t->CARR_PHASE_FRAC;

  s->code_phase += t->LENGTH * t->CODE_PINC;

  *count_snapshot = t->START_SNAPSHOT;
  *code_phase_early = (double)nap_code_phase /
                          NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP;
  *carrier_phase = (double)-nap_carr_phase /
                       NAP_TRACK_CARRIER_PHASE_UNITS_PER_CYCLE;
}

void nap_track_disable(u8 channel)
{
  NAP->TRK_CONTROL &= ~(1 << channel);
}


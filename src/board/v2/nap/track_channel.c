/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
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

#include "libswiftnav/prns.h"
#include "libswiftnav/constants.h"

/** \addtogroup nap
 * \{ */

/** \defgroup track_channel Track Channel
 * Interface to the SwiftNAP track channels.
 * \{ */

/* SPI register IDs */
#define NAP_REG_TRACK_BASE           0x0A
#define NAP_TRACK_N_REGS             5
#define NAP_REG_TRACK_INIT_OFFSET    0x00
#define NAP_REG_TRACK_UPDATE_OFFSET  0x01
#define NAP_REG_TRACK_CORR_OFFSET    0x02
#define NAP_REG_TRACK_PHASE_OFFSET   0x03
#define NAP_REG_TRACK_CODE_OFFSET    0x04

/** Number of tracking channels.
 * Actual number of track channels NAP configuration was built with. Read from
 * configuration flash at runtime with nap_conf_rd_parameters().
 */
u8 nap_track_n_channels;

/** Pack data for writing to a NAP track channel's INIT register.
 *
 * \param pack          Array of u8 to pack data into.
 * \param prn           CA code PRN number (0-31) to track. (deprecated)
 * \param carrier_phase Initial carrier phase.
 * \param code_phase    Initial code phase.
 */
static void nap_track_init_pack(u8 pack[], u8 prn, s32 carrier_phase, u16 code_phase)
{
  /* TODO: No longer need to write PRN. */
  pack[0] = ((code_phase << 5) >> 16) & 0x07;
  pack[1] = (code_phase << 5) >> 8;
  pack[2] = (((carrier_phase << 5) >> 24) & 0x1F) | (code_phase << 5);
  pack[3] = (carrier_phase << 5) >> 16;
  pack[4] = (carrier_phase << 5) >> 8;
  pack[5] = (prn & 0x1F) | (carrier_phase << 5 & 0xE0);
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
static float propagate_code_phase(float code_phase, float carrier_freq, u32 n_samples)
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
 */
void nap_track_init(u8 channel, gnss_signal_t sid, u32 ref_timing_count,
                    float carrier_freq, float code_phase)
{
  ref_timing_count -= 0.5*16;

  u32 track_count = nap_timing_count() + 20000;
  float cp = propagate_code_phase(code_phase, carrier_freq,
                                  track_count - ref_timing_count);

  /* Contrive for the timing strobe to occur at or close to a
   * PRN edge (code phase = 0) */
  track_count += 16 * (1023.0-cp) * (1.0 + carrier_freq / GPS_L1_HZ);

  nap_xfer_blocking(NAP_REG_TRACK_BASE + channel * NAP_TRACK_N_REGS
                     + NAP_REG_TRACK_CODE_OFFSET, 128, 0, ca_code(sid));

  u8 temp[6] = { 0, 0, 0, 0, 0, 0 };
  nap_track_init_pack(temp, 0, 0, 0);
  nap_xfer_blocking(NAP_REG_TRACK_BASE + channel * NAP_TRACK_N_REGS
                     + NAP_REG_TRACK_INIT_OFFSET, 6, 0, temp);

  double cp_rate = (1 + carrier_freq/GPS_L1_HZ) * GPS_CA_CHIPPING_RATE;
  nap_track_update_wr_blocking(channel,
    carrier_freq * NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ,
    cp_rate * NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ,
    0, 0
  );

  /* Schedule the timing strobe for start_sample_count. */
  nap_timing_strobe(track_count);
  nap_timing_strobe_wait(100);
}

/** Pack data for writing to a NAP track channel's UPDATE register.
 *
 * \param pack            Array of u8 to pack data into.
 * \param carrier_freq    Next correlation period's carrier frequency.
 * \param code_phase_rate Next correlation period's code phase rate.
 */
static void nap_track_update_pack(u8 pack[], s32 carrier_freq, u32 code_phase_rate,
                           u8 rollover_count, u8 corr_spacing)
{
  pack[0] = (code_phase_rate >> 24) & 0x1F;
  pack[1] = (code_phase_rate >> 16);
  pack[2] = (code_phase_rate >> 8);
  pack[3] = code_phase_rate;
  pack[4] = ((corr_spacing << 1) & 0xFE) | ((carrier_freq >> 16) & 0x01);
  pack[5] = carrier_freq >> 8;
  pack[6] = carrier_freq;
  pack[7] = rollover_count;
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
 */
void nap_track_update_wr_blocking(u8 channel, s32 carrier_freq,
                                  u32 code_phase_rate, u8 rollover_count,
                                  u8 corr_spacing)
{
  u8 temp[8] = { 0 };

  nap_track_update_pack(temp, carrier_freq, code_phase_rate,
                        rollover_count, corr_spacing);
  nap_xfer_blocking(NAP_REG_TRACK_BASE + channel * NAP_TRACK_N_REGS
                     + NAP_REG_TRACK_UPDATE_OFFSET, 8, 0, temp);
}

/** Unpack data read from a NAP track channel's CORR register.
 *
 * \param packed       Array of u8 data read from channnel's CORR register.
 * \param sample_count Number of sample clock cycles in correlation period.
 * \param corrs        Array of E,P,L correlations from correlation period.
 */
static void nap_track_corr_unpack(u8 packed[], u32* sample_count, corr_t corrs[])
{
  /* graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend */

  struct { s32 xtend : 24; } sign;

  *sample_count = (packed[0] << 16) | (packed[1] << 8) | packed[2];

  for (u8 i = 0; i < 3; i++) {

    sign.xtend  = (packed[6 * (3 - i - 1) + 3] << 16) /* MSB */
                | (packed[6 * (3 - i - 1) + 4] << 8)  /* Middle byte */
                | (packed[6 * (3 - i - 1) + 5]);      /* LSB */

    corrs[i].Q = sign.xtend;  /* Sign extend! */

    sign.xtend  = (packed[6 * (3 - i - 1) + 6] << 16) /* MSB */
                | (packed[6 * (3 - i - 1) + 7] << 8)  /* Middle byte */
                | (packed[6 * (3 - i - 1) + 8]);      /* LSB */

    corrs[i].I = sign.xtend;  /* Sign extend! */
  }
}

/** Read data from a NAP track channel's CORR register.
 *
 * \param channel      NAP track channel whose CORR register to read.
 * \param sample_count Number of sample clock cycles in correlation period.
 * \param corrs        Array of E,P,L correlations from correlation period.
 */
void nap_track_corr_rd_blocking(u8 channel, u32* sample_count, corr_t corrs[])
{
  /* 2 (I or Q) * 3 (E, P or L) * 3 (24 bits / 8) + 24 bits sample count. */
  u8 temp[2*3*3 + 3];

  nap_xfer_blocking(NAP_REG_TRACK_BASE + channel * NAP_TRACK_N_REGS
                     + NAP_REG_TRACK_CORR_OFFSET, 2*3*3 + 3, temp, temp);
  nap_track_corr_unpack(temp, sample_count, corrs);
}

/** Unpack data read from a NAP track channel's PHASE register.
 *
 * \param packed        Array of u8 data read from channnel's PHASE register.
 * \param carrier_phase Carrier phase at end of last correlation period.
 * \param code_phase    Prompt code phase at end of last correlation period.
 *                      (deprecated, is always zero, as prompt code phase
 *                      rollovers are defined to be edges of correlation
 *                      period)
 */
/* TODO : take code phase out of phase register, it's always zero */
static void nap_track_phase_unpack(u8 packed[], s32* carrier_phase, u64* code_phase)
{
  /* graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend */

  struct { s32 xtend : 24; } sign;

  sign.xtend = packed[8] |
               (packed[7] << 8) |
               (packed[6] << 16);

  *carrier_phase = sign.xtend; /* Sign extend! */

  *code_phase = (u64)packed[5] |
                ((u64)packed[4] << 8) |
                ((u64)packed[3] << 16) |
                ((u64)packed[2] << 24) |
                ((u64)packed[1] << 32) |
                ((u64)packed[0] << 40);
}

/** Read data from a NAP track channel's PHASE register.
 *
 * \param channel       NAP track channel whose PHASE register to read.
 * \param carrier_phase Carrier phase at end of last correlation period.
 * \param code_phase    Prompt code phase at end of last correlation period.
 *                      (deprecated, is always zero, as prompt code phase
 *                      rollovers are defined to be edges of correlation
 *                      period)
 */
void nap_track_phase_rd_blocking(u8 channel, s32* carrier_phase,
                                 u64* code_phase)
{
  u8 temp[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  nap_xfer_blocking(NAP_REG_TRACK_BASE + channel * NAP_TRACK_N_REGS
                     + NAP_REG_TRACK_PHASE_OFFSET, 9, temp, temp);
  nap_track_phase_unpack(temp, carrier_phase, code_phase);
}

void nap_track_disable(u8 channel)
{
  nap_track_update_wr_blocking(channel, 0, 0, 0, 0);
}

/** \} */

/** \} */


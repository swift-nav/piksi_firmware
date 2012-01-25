/*
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "swift_nap_io.h"
#include "track.h"

tracking_channel_t tracking_channel[TRACK_N_CHANNELS];

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
  u32 code_phase_rate = (1.0 + carrier_freq/L1_HZ) * TRACK_NOMINAL_CODE_PHASE_RATE;

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
 * Initialises a tracking channel on the Swift NAP.
 *
 * \param prn                PRN number - 1 (0-31).
 * \param channel            Tracking channel number on the Swift NAP.
 * \param code_phase         Code phase at start of tracking in chips [0-1023).
 * \param carrier_freq       Carrier frequency (Doppler) at start of tracking in Hz.
 * \param start_sample_count Sample count on which to start tracking.
 */
void tracking_channel_init(u8 channel, u8 prn, float code_phase, float carrier_freq, u32 start_sample_count)
{
  /* Calculate the code phase rate with carrier aiding. */
  u32 code_phase_rate = (1 + carrier_freq/L1_HZ) * TRACK_NOMINAL_CODE_PHASE_RATE;

  /* Setup tracking_channel struct. */
  tracking_channel[channel].state = TRACKING_FIRST_LOOP;
  tracking_channel[channel].channel_num = channel;
  tracking_channel[channel].prn = prn;
  tracking_channel[channel].dll_disc = 0;
  tracking_channel[channel].pll_disc = 0;
  tracking_channel[channel].dll_freq = code_phase_rate / TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;
  tracking_channel[channel].pll_freq = carrier_freq;

  /* TODO: Write PRN into tracking channel when the FPGA code supports this. */

  /* Starting carrier phase is set to zero as we don't 
   * know the carrier freq well enough to calculate it.
   */
  track_write_init(channel, prn, 0, code_phase*TRACK_CODE_PHASE_UNITS_PER_CHIP);
  track_write_update(channel, carrier_freq*TRACK_CARRIER_FREQ_UNITS_PER_HZ, code_phase_rate);

  /* Schedule the timing strobe for start_sample_count. */
  timing_strobe(start_sample_count);
}

void tracking_channel_update(tracking_channel_t* chan)
{
  switch(chan->state)
  {
  case TRACKING_FIRST_LOOP:
    /* First set of correlations are junk so do one open loop update. */
    track_write_update(chan->channel_num, \
                       chan->pll_freq*TRACK_CARRIER_FREQ_UNITS_PER_HZ, \
                       chan->dll_freq*TRACK_CODE_PHASE_RATE_UNITS_PER_HZ);

    /* Next update start running loop filters. */
    chan->state = TRACKING_RUNNING;
    break;

  case TRACKING_RUNNING:
  {
    /*corr_t cs[3];*/
    corr_t* cs = chan->cs;

    /* Read early ([0]), prompt ([1]) and late ([2]) correlations. */
    track_read_corr(chan->channel_num, cs);

    /* Update I and Q magnitude filters for SNR calculation.
     * filter = (1 - 2^-FILTER_COEFF)*filter + correlation_magnitude
     */
    chan->I_filter -= chan->I_filter >> I_FILTER_COEFF;
    chan->I_filter += abs(cs[1].I);
    chan->Q_filter -= chan->Q_filter >> Q_FILTER_COEFF;
    chan->Q_filter += abs(cs[1].Q);
    chan->snr = (float)(chan->I_filter >> I_FILTER_COEFF) / (chan->Q_filter >> Q_FILTER_COEFF);

    double dll_disc_prev = chan->dll_disc;
    double pll_disc_prev = chan->pll_disc;

    /* TODO: check for divide by zero. */
    chan->pll_disc = atan((double)cs[1].Q/cs[1].I)/(2*PI);

    chan->pll_freq += PLL_PGAIN*(chan->pll_disc-pll_disc_prev) + PLL_IGAIN*chan->pll_disc;

    double early_mag = sqrt((double)cs[0].I*cs[0].I + (double)cs[0].Q*cs[0].Q);
    double late_mag = sqrt((double)cs[2].I*cs[2].I + (double)cs[2].Q*cs[2].Q);

    chan->dll_disc = (early_mag - late_mag) / (early_mag + late_mag);

    chan->dll_freq += DLL_PGAIN*(chan->dll_disc-dll_disc_prev) + DLL_IGAIN*chan->dll_disc;

    track_write_update(chan->channel_num, \
                       chan->pll_freq*TRACK_CARRIER_FREQ_UNITS_PER_HZ, \
                       chan->dll_freq*TRACK_CODE_PHASE_RATE_UNITS_PER_HZ);
    break;
  }
  case TRACKING_DISABLED:
  default:
    /* WTF? */
    break;

  }
}




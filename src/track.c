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

#include "main.h"
#include "swift_nap_io.h"

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
void tracking_channel_init(u8 prn, u8 channel, float code_phase, float carrier_freq, u32 start_sample_count)
{
  /* Calculate the code phase rate with carrier aiding. */
  u32 code_phase_rate = (1 + carrier_freq/L1_HZ) * TRACK_NOMINAL_CODE_PHASE_RATE;

  /* TODO: Write PRN into tracking channel when the FPGA code supports this. */

  /* Starting carrier phase is set to zero as we don't 
   * know the carrier freq well enough to calculate it.
   */
  track_write_init(channel, prn, 0, code_phase*TRACK_CODE_PHASE_UNITS_PER_CHIP);
  track_write_update(channel, carrier_freq*TRACK_CARRIER_FREQ_UNITS_PER_HZ, code_phase_rate);

  /* Schedule the timing strobe for start_sample_count. */
  timing_strobe(start_sample_count);
}


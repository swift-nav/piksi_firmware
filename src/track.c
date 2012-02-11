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
#include <string.h>
#include <libopencm3/stm32/f2/gpio.h>

#include "main.h"
#include "swift_nap_io.h"
#include "track.h"
#include "nav_msg.h"

/* Initialiser using GNU extension, see
 * http://gcc.gnu.org/onlinedocs/gcc/Designated-Inits.html 
 * tracking_channel_t tracking_channel[TRACK_N_CHANNELS] = \
 *   {[0 ... TRACK_N_CHANNELS-1] = {.state = TRACKING_DISABLED}};
 */

/* Initialiser actually not needed, static array inits to zero
 * and TRACKING_DISABLED = 0.
 */
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
  /* Calculate code phase rate with carrier aiding. */
  float code_phase_rate = (1 + carrier_freq/L1_HZ) * NOMINAL_CODE_PHASE_RATE_HZ;

  /* Setup tracking_channel struct. */
  tracking_channel[channel].state = TRACKING_FIRST_LOOP;
  tracking_channel[channel].prn = prn;
  tracking_channel[channel].update_count = 0;
  tracking_channel[channel].TOW_ms = 0;
  tracking_channel[channel].snr_threshold_count = 0;
  tracking_channel[channel].dll_disc = 0;
  tracking_channel[channel].pll_disc = 0;
  tracking_channel[channel].I_filter = 0;
  tracking_channel[channel].Q_filter = 0;
  tracking_channel[channel].code_phase_rate = code_phase_rate;
  tracking_channel[channel].carrier_freq = carrier_freq;
  
  nav_msg_init(&tracking_channel[channel].nav_msg);

  /* TODO: Write PRN into tracking channel when the FPGA code supports this. */

  /* Starting carrier phase is set to zero as we don't 
   * know the carrier freq well enough to calculate it.
   */
  track_write_init_blocking(channel, prn, 0, code_phase*TRACK_CODE_PHASE_UNITS_PER_CHIP);
  track_write_update_blocking(channel, \
                     carrier_freq*TRACK_CARRIER_FREQ_UNITS_PER_HZ, \
                     code_phase_rate*TRACK_CODE_PHASE_RATE_UNITS_PER_HZ);

  /* Schedule the timing strobe for start_sample_count. */
  timing_strobe(start_sample_count);
}

void tracking_channel_get_corrs(u8 channel)
{
  gpio_set(GPIOC, GPIO10);
  tracking_channel_t* chan = &tracking_channel[channel];

  switch(chan->state)
  {
  case TRACKING_FIRST_LOOP: {
    /* First set of correlations are junk so do one open loop update. */
    track_read_corr_blocking(channel, chan->cs);
    break;
  }
  case TRACKING_RUNNING:
    /* Read early ([0]), prompt ([1]) and late ([2]) correlations. */
    track_read_corr_blocking(channel, chan->cs);
    break;

  case TRACKING_DISABLED:
  default:
    /* WTF? */
    break;
  }
  gpio_clear(GPIOC, GPIO10);
}

void tracking_channel_update(u8 channel)
{
  gpio_set(GPIOC, GPIO11);
  tracking_channel_t* chan = &tracking_channel[channel];

  switch(chan->state)
  {
  case TRACKING_FIRST_LOOP:
    chan->update_count++;

    /* First set of correlations are junk so do one open loop update. */
    track_write_update_blocking(channel, \
                       chan->carrier_freq*TRACK_CARRIER_FREQ_UNITS_PER_HZ, \
                       chan->code_phase_rate*TRACK_CODE_PHASE_RATE_UNITS_PER_HZ);

    /* Next update start running loop filters. */
    chan->state = TRACKING_RUNNING;
    break;

  case TRACKING_RUNNING:
  {
    chan->update_count++;
    chan->TOW_ms++;
    if (chan->TOW_ms == 7*24*60*60*1000)
      chan->TOW_ms = 0;

    /* Correlations should already be in chan->cs thanks to
     * tracking_channel_get_corrs.
     */
    corr_t* cs = chan->cs;

    /* Update I and Q magnitude filters for SNR calculation.
     * filter = (1 - 2^-FILTER_COEFF)*filter + correlation_magnitude
     * If filters are uninitialised (=0) then initialise them with the
     * first set of corellations.
     */
    if (chan->I_filter == 0 && chan->Q_filter == 0) {
      chan->I_filter = abs(cs[1].I) << I_FILTER_COEFF;
      chan->Q_filter = abs(cs[1].Q) << Q_FILTER_COEFF;
    } else {
      chan->I_filter -= chan->I_filter >> I_FILTER_COEFF;
      chan->I_filter += abs(cs[1].I);
      chan->Q_filter -= chan->Q_filter >> Q_FILTER_COEFF;
      chan->Q_filter += abs(cs[1].Q);
    }
    
    /* Accumulate code phase, taking care to try and exactly match what is
     * going on in the Swift NAP. As we are interrupted on code phase rollover
     * in the Swift NAP and that is when all the correlations are valid we must
     * find the residual (sub-sample) code phase after rollover.
     *
     * i.e. solve N*code_phase_rate > 1 PRN == 2^42 in Swift NAP tracking units.
     *
     * NOTE: We can save a lot of time here by realising that with reasonable
     * Doppler shifts N can never be more than 1 sample away from the nominal
     * value (16*1023), hence we only need to test these cases.
     */

    /* trial_cp = 16*1023*code_phase_rate_fp */
    u64 trial_cp = ((chan->code_phase_rate_fp << 10) - chan->code_phase_rate_fp) << 4;
    /* If 16*1023*CPR > 2^42 */
    if (trial_cp > ((u64)1<<42)) {
      /* If (16*1023-1)*CPR > 2^42) */
      if (trial_cp-chan->code_phase_rate_fp > ((u64)1<<42))
        /* Then rollover was after 16*1023-1 samples, code_phase = (16*1023-1)*CPR % 2^42 */
        chan->code_phase = (trial_cp-chan->code_phase_rate_fp) & (((u64)1<<42)-1);
      else
        /* Then rollover was after 16*1023 samples, code_phase = 16*1023*CPR % 2^42 */
        chan->code_phase = trial_cp & (((u64)1<<42)-1);
    } else 
      /* Then rollover was after 16*1023+1 samples, code_phase = (16*1023+1)*CPR % 2^42 */
      chan->code_phase = (trial_cp+chan->code_phase_rate_fp) & (((u64)1<<42)-1);

    /* Run the loop filters. */

    double dll_disc_prev = chan->dll_disc;
    double pll_disc_prev = chan->pll_disc;

    gpio_toggle(GPIOC, GPIO11);
    /* TODO: check for divide by zero. */
    chan->pll_disc = atan((double)cs[1].Q/cs[1].I)/(2*PI);
    gpio_toggle(GPIOC, GPIO11);

    chan->carrier_freq += PLL_PGAIN*(chan->pll_disc-pll_disc_prev) + PLL_IGAIN*chan->pll_disc;

    double early_mag = sqrt((double)cs[0].I*cs[0].I + (double)cs[0].Q*cs[0].Q);
    double late_mag = sqrt((double)cs[2].I*cs[2].I + (double)cs[2].Q*cs[2].Q);

    chan->dll_disc = (early_mag - late_mag) / (early_mag + late_mag);

    chan->code_phase_rate += DLL_PGAIN*(chan->dll_disc-dll_disc_prev) + DLL_IGAIN*chan->dll_disc;
 

    u32 timer_val = timing_count();
    static u32 max_timer_val = 0;

    u32 TOW_ms = nav_msg_update(&chan->nav_msg, cs[1].I);
    
    timer_val = timing_count() - timer_val;
    if (timer_val > max_timer_val) {
      max_timer_val = timer_val;
      printf("n_m_u took %.1f us\n", max_timer_val/16.368);
    }

    if (chan->TOW_ms != TOW_ms) {
      printf("PRN %d TOW mismatch: %u, %u\n",(int)chan->prn, (unsigned int)chan->TOW_ms, (unsigned int)TOW_ms);
      chan->TOW_ms = TOW_ms;
    }

    /* Save the exact code phase rate in fixed point as used by the Swift NAP
     * so we can exactly predict rollover / accumulate the code phase on the
     * STM next tracking update.
     */
    chan->code_phase_rate_fp = chan->code_phase_rate*TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;

    track_write_update_blocking(channel, \
                       chan->carrier_freq*TRACK_CARRIER_FREQ_UNITS_PER_HZ, \
                       chan->code_phase_rate*TRACK_CODE_PHASE_RATE_UNITS_PER_HZ);
    break;
  }
  case TRACKING_DISABLED:
  default:
    /* WTF? */
    tracking_channel_disable(channel);
    break;
  }
  gpio_clear(GPIOC, GPIO11);
}

void tracking_channel_disable(u8 channel)
{
  /* Write zero to the code phase rate to stop the channel
   * from generating interrupts.
   */
  track_write_update_blocking(channel, 0, 0);
  tracking_channel[channel].state = TRACKING_DISABLED;
}


float tracking_channel_snr(u8 channel)
{
  /* Calculate SNR from I and Q filtered magnitudes. */ 
  return (float)(tracking_channel[channel].I_filter >> I_FILTER_COEFF) / \
                (tracking_channel[channel].Q_filter >> Q_FILTER_COEFF);
}



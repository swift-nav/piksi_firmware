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
 * Initialises a tracking channel on the Swift NAP. The start_sample_count
 * must be contrived to be at or close to a PRN edge (code phase = 0).
 *
 * \param prn                PRN number - 1 (0-31).
 * \param channel            Tracking channel number on the Swift NAP.
 * \param carrier_freq       Carrier frequency (Doppler) at start of tracking in Hz.
 * \param start_sample_count Sample count on which to start tracking.
 */
void tracking_channel_init(u8 channel, u8 prn, float carrier_freq, u32 start_sample_count)
{
  /* Calculate code phase rate with carrier aiding. */
  float code_phase_rate = (1 + carrier_freq/L1_HZ) * NOMINAL_CODE_PHASE_RATE_HZ;

  /* Setup tracking_channel struct. */
  tracking_channel[channel].state = TRACKING_RUNNING;
  tracking_channel[channel].prn = prn;
  tracking_channel[channel].update_count = 0;
  tracking_channel[channel].TOW_ms = 0;
  tracking_channel[channel].snr_threshold_count = 0;
  tracking_channel[channel].dll_disc = 0;
  tracking_channel[channel].pll_disc = 0;
  tracking_channel[channel].I_filter = 0;
  tracking_channel[channel].Q_filter = 0;
  tracking_channel[channel].code_phase_early = 0;
  tracking_channel[channel].code_phase_rate_fp = code_phase_rate*TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;
  tracking_channel[channel].code_phase_rate_fp_prev[0] = tracking_channel[channel].code_phase_rate_fp;
  tracking_channel[channel].code_phase_rate_fp_prev[1] = tracking_channel[channel].code_phase_rate_fp;
  tracking_channel[channel].code_phase_rate = code_phase_rate;
  tracking_channel[channel].carrier_freq = carrier_freq;
  tracking_channel[channel].sample_count = start_sample_count;

  nav_msg_init(&tracking_channel[channel].nav_msg);

  /* TODO: Write PRN into tracking channel when the FPGA code supports this. */

  /* Starting carrier phase is set to zero as we don't 
   * know the carrier freq well enough to calculate it.
   */
  /* TODO: add comment explaining why code phase is set to zero. See doxygen comment up top */
  track_write_init_blocking(channel, prn, 0, 0);
  track_write_update_blocking(channel, \
                     carrier_freq*TRACK_CARRIER_FREQ_UNITS_PER_HZ, \
                     tracking_channel[channel].code_phase_rate_fp);

  /* Schedule the timing strobe for start_sample_count. */
  timing_strobe(start_sample_count);
}

void tracking_channel_get_corrs(u8 channel)
{
  gpio_set(GPIOC, GPIO10);
  tracking_channel_t* chan = &tracking_channel[channel];

  switch(chan->state)
  {
    case TRACKING_RUNNING:
      /* Read early ([0]), prompt ([1]) and late ([2]) correlations. */
      track_read_corr_blocking(channel, &chan->corr_sample_count, chan->cs);
      break;

    case TRACKING_DISABLED:
    default:
      /* TODO: WTF? */
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
    case TRACKING_RUNNING:
    {
      chan->update_count++;
      chan->sample_count += chan->corr_sample_count;
      chan->TOW_ms++;
      if (chan->TOW_ms == 7*24*60*60*1000)
        chan->TOW_ms = 0;

      chan->code_phase_early += chan->corr_sample_count*chan->code_phase_rate_fp_prev[1];
      chan->code_phase_prompt = chan->code_phase_early - 8*chan->code_phase_rate_fp_prev[1];
      /*u64 cp;*/
      /*u32 cf;*/
      /*DO_ONLY(10,*/
        /*track_read_phase_blocking(channel, &cf, &cp);*/
        /*printf("%d CPR: 0x%08X, count: %d, NAP: 0x%08X, STM: 0x%08X\n", chan->prn+1, (unsigned int)chan->code_phase_rate_fp_prev[1], (unsigned int)chan->corr_sample_count, (unsigned int)(cp&0xFFFFFFFF), (unsigned int)chan->code_phase_early);*/
      /*)*/

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
      
      u32 lag = timing_count() - chan->sample_count;
      if (lag > 16368)
        printf("PRN %02d, lag = %u samples\n",chan->prn+1, (unsigned int)lag);

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

      if (TOW_ms && chan->TOW_ms != TOW_ms) {
        printf("PRN %d TOW mismatch: %u, %u\n",(int)chan->prn + 1, (unsigned int)chan->TOW_ms, (unsigned int)TOW_ms);
        chan->TOW_ms = TOW_ms;
      }

      chan->code_phase_rate_fp = chan->code_phase_rate*TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;
      chan->code_phase_rate_fp_prev[1] = chan->code_phase_rate_fp_prev[0];
      chan->code_phase_rate_fp_prev[0] = chan->code_phase_rate_fp;

      track_write_update_blocking(channel, \
                         chan->carrier_freq*TRACK_CARRIER_FREQ_UNITS_PER_HZ, \
                         chan->code_phase_rate_fp);
      break;
    }
    case TRACKING_DISABLED:
    default:
      /* TODO: WTF? */
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



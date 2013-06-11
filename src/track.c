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
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/exti.h>

#include "main.h"
#include "board/nap/track_channel.h"
#include "sbp.h"
#include "track.h"

#include <libswiftnav/pvt.h>

/** \addtogroup manage
 * \{ */

/** \defgroup track Track
 * Functions, structs, and interrupt service routines for managing tracking.
 * \{ */

/* Initialiser using GNU extension, see
 * http://gcc.gnu.org/onlinedocs/gcc/Designated-Inits.html
 * tracking_channel_t tracking_channel[NAP_MAX_N_TRACK_CHANNELS] = \
 *   {[0 ... nap_track_n_channels-1] = {.state = TRACKING_DISABLED}};
 */

/* Initialiser actually not needed, static array inits to zero
 * and TRACKING_DISABLED = 0.
 */
tracking_channel_t tracking_channel[NAP_MAX_N_TRACK_CHANNELS];

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
  u32 code_phase_rate = (1.0 + carrier_freq/L1_HZ) * NAP_TRACK_NOMINAL_CODE_PHASE_RATE;

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
 * must be contrived to be at or close to a PRN edge (PROMPT code phase = 0).
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

  /* Adjust the channel start time as the start_sample_count passed
   * in corresponds to a PROMPT code phase rollover but we want to
   * start the channel on an EARLY code phase rollover.
   */
  start_sample_count -= 0.5*16;

  /* Setup tracking_channel struct. */
  tracking_channel[channel].state = TRACKING_RUNNING;
  tracking_channel[channel].prn = prn;
  tracking_channel[channel].update_count = 0;
  /* Use -1 to indicate an uninitialised value. */
  tracking_channel[channel].TOW_ms = -1;
  tracking_channel[channel].snr_threshold_count = 0;

  comp_tl_init(&(tracking_channel[channel].tl_state), 1e3,
               code_phase_rate-1.023e6, 2, 0.7, 1,
               carrier_freq, 25, 0.7, 0.25,
               0.005, 1540, 5000);

  tracking_channel[channel].I_filter = 0;
  tracking_channel[channel].Q_filter = 0;
  tracking_channel[channel].code_phase_early = 0;
  tracking_channel[channel].code_phase_rate_fp = code_phase_rate*NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;
  tracking_channel[channel].code_phase_rate_fp_prev = tracking_channel[channel].code_phase_rate_fp;
  tracking_channel[channel].code_phase_rate = code_phase_rate;
  tracking_channel[channel].carrier_freq = carrier_freq;
  tracking_channel[channel].sample_count = start_sample_count;

  nav_msg_init(&tracking_channel[channel].nav_msg);

  /* Starting carrier phase is set to zero as we don't
   * know the carrier freq well enough to calculate it.
   */
  /* Start with code phase of zero as we have conspired for the
   * channel to be initialised on an EARLY code phase rollover.
   */
  nap_track_code_wr_blocking(channel, prn);
  nap_track_init_wr_blocking(channel, prn, 0, 0);
  nap_track_update_wr_blocking(channel, \
                     carrier_freq*NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ, \
                     tracking_channel[channel].code_phase_rate_fp);

  /* Schedule the timing strobe for start_sample_count. */
  nap_timing_strobe(start_sample_count);
}

void tracking_channel_get_corrs(u8 channel)
{
  gpio_set(GPIOC, GPIO10);
  tracking_channel_t* chan = &tracking_channel[channel];

  switch(chan->state)
  {
    case TRACKING_RUNNING:
      /* Read early ([0]), prompt ([1]) and late ([2]) correlations. */
      nap_track_corr_rd_blocking(channel, &chan->corr_sample_count, chan->cs);
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
      if (chan->TOW_ms > 0) {
        chan->TOW_ms++;
        if (chan->TOW_ms == 7*24*60*60*1000)
          chan->TOW_ms = 0;
      }

      chan->code_phase_early = (u64)chan->code_phase_early + (u64)chan->corr_sample_count*chan->code_phase_rate_fp_prev;

      /*u64 cp;*/
      /*u32 cf;*/
      /*nap_track_phase_rd_blocking(channel, &cf, &cp);*/
      /*if ((cp&0xFFFFFFFF) != chan->code_phase_early) {*/
        /*DO_ONLY(100,*/
          /*printf("%d %u CPR: 0x%08X, count: %d, NAP: 0x%011llX, STM: 0x%08X\n", chan->prn+1, (unsigned int)chan->update_count, (unsigned int)chan->code_phase_rate_fp_prev, (unsigned int)chan->corr_sample_count, (unsigned long long)cp, (unsigned int)chan->code_phase_early);*/
        /*);*/
      /*}*/

      /* Correlations should already be in chan->cs thanks to
       * tracking_channel_get_corrs.
       */
      corr_t* cs = chan->cs;

      /* Update I and Q magnitude filters for SNR calculation.
       * filter = (1 - 2^-FILTER_COEFF)*filter + correlation_magnitude
       * If filters are uninitialised (=0) then initialise them with the
       * first set of correlations.
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

      /* Run the loop filters. */

      /* TODO: Make this more elegant. */
      correlation_t cs2[3];
      for (u32 i = 0; i < 3; i++) {
        cs2[i].I = cs[2-i].I;
        cs2[i].Q = cs[2-i].Q;
      }
      comp_tl_update(&(chan->tl_state), cs2);
      chan->carrier_freq = chan->tl_state.carr_freq;
      chan->code_phase_rate = chan->tl_state.code_freq + 1.023e6;

      s32 TOW_ms = nav_msg_update(&chan->nav_msg, cs[1].I);

      if (TOW_ms > 0 && chan->TOW_ms != TOW_ms) {
        printf("PRN %d TOW mismatch: %u, %u\n",(int)chan->prn + 1, (unsigned int)chan->TOW_ms, (unsigned int)TOW_ms);
        chan->TOW_ms = TOW_ms;
      }

      chan->code_phase_rate_fp_prev = chan->code_phase_rate_fp;
      chan->code_phase_rate_fp = chan->code_phase_rate*NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;

      nap_track_update_wr_blocking(channel, \
                         chan->carrier_freq*NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ, \
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
  nap_track_update_wr_blocking(channel, 0, 0);
  tracking_channel[channel].state = TRACKING_DISABLED;
}

void tracking_update_measurement(u8 channel, channel_measurement_t *meas)
{
  tracking_channel_t* chan = &tracking_channel[channel];

  /* Update our channel measurement. */
  meas->prn = chan->prn;
  meas->code_phase_chips = (double)chan->code_phase_early / NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP;
  //meas->code_phase_rate = (double)chan->code_phase_rate_fp_prev / NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;
  //meas->code_phase_rate = 1.023e6 * (1 + chan->carrier_freq/L1_HZ);
  meas->code_phase_rate = chan->code_phase_rate;
  meas->carrier_phase = 0;
  meas->carrier_freq = chan->carrier_freq;
  meas->time_of_week_ms = chan->TOW_ms;
  meas->receiver_time = (double)chan->sample_count / SAMPLE_FREQ;
  meas->snr = tracking_channel_snr(channel);
}

float tracking_channel_snr(u8 channel)
{
  /* Calculate SNR from I and Q filtered magnitudes. */
  return (float)(tracking_channel[channel].I_filter >> I_FILTER_COEFF) / \
                (tracking_channel[channel].Q_filter >> Q_FILTER_COEFF);
}

void tracking_send_state()
{
  tracking_state_msg_t states[nap_track_n_channels];
  for (u8 i=0; i<nap_track_n_channels; i++) {
    states[i].state = tracking_channel[i].state;
    states[i].prn = tracking_channel[i].prn;
    if (tracking_channel[i].state == TRACKING_RUNNING)
      states[i].cn0 = tracking_channel_snr(i);
    else
      states[i].cn0 = -1;
  }
  sbp_send_msg(MSG_TRACKING_STATE, sizeof(states), (u8*)states);
}

/** \} */

/** \} */

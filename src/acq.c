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
#include <libopencm3/stm32/f4/gpio.h>

#include "board/nap/acq_channel.h"
#include "acq.h"
#include "track.h"

/** \defgroup acq Acquisition
 * Do acquisition searches via interrupt driven scheduling of SwiftNAP
 * acquisition channel correlations and peak detection.
 * \{ */

acq_state_t acq_state;

/** Schedule a load of samples into the acquisition channel's sample ram.
 * The load starts at the end of the next timing strobe and continues until the
 * ram is full, at which time an interrupt is raised to the STM. This interrupt
 * is cleared by clearing the load enable bit of the acquisition channel's
 * LOAD ENABLE register.
 *
 * \param count The value of the NAP's internal counter at which the timing
 *              strobe is to go low.
 */
void acq_schedule_load(u32 count)
{
  acq_state.state = ACQ_LOADING;
  nap_acq_load_wr_enable_blocking();
  nap_timing_strobe(count);
}

/** Handle an acquisition load done interrupt from the NAP acquisition channel.
 * Clear the enable bit of the acquisition channel load register and change
 * the acquisition state to ACQ_LOADING_DONE.
 */
void acq_service_load_done()
{
  nap_acq_load_wr_disable_blocking();
  acq_state.state = ACQ_LOADING_DONE;
}

/** Query the state of the acquisition channel sample ram loading.
 * \return 1 if loading has finished, 0 otherwise
 */
u8 acq_get_load_done()
{
  return (acq_state.state == ACQ_LOADING_DONE);
}

/** Start acquisitions for a PRN over a code phase / carrier frequency range.
 * Translate the passed code phase and carrier frequency float values into
 * acquisition register values. Write values for the first acquisition to the
 * channel, and then write values for the next pipelined acquisition.
 *
 * \param prn      PRN to search (nap_acq_code_wr_blocking must be called prior)
 * \param cp_min   Starting code phase of the first acquisition. (chips)
 * \param cp_max   Starting code phase of the last acquisition. (chips)
 * \param cf_min   Carrier frequency of the first acquisition. (Hz)
 * \param cf_max   Carrier frequency of the last acquisition. (Hz)
 * \param cf_bin_width Step size between each carrier frequency to search. (Hz)
 */
void acq_start(u8 prn, float cp_min, float cp_max, float cf_min, float cf_max, float cf_bin_width)
{
  /* Calculate the range parameters in acq units. Explicitly expand
   * the range to the nearest multiple of the step size to make sure
   * we cover at least the specified range.
   */
  acq_state.cf_step = cf_bin_width*NAP_ACQ_CARRIER_FREQ_UNITS_PER_HZ;
  acq_state.cf_min = acq_state.cf_step*floor(cf_min*NAP_ACQ_CARRIER_FREQ_UNITS_PER_HZ / (float)acq_state.cf_step);
  acq_state.cf_max = acq_state.cf_step*ceil(cf_max*NAP_ACQ_CARRIER_FREQ_UNITS_PER_HZ / (float)acq_state.cf_step);
  /* cp_step = nap_acq_n_taps */
  acq_state.cp_min = nap_acq_n_taps*floor(cp_min*NAP_ACQ_CODE_PHASE_UNITS_PER_CHIP / (float)nap_acq_n_taps);
  acq_state.cp_max = nap_acq_n_taps*ceil(cp_max*NAP_ACQ_CODE_PHASE_UNITS_PER_CHIP / (float)nap_acq_n_taps);


  /* Initialise our acquisition state struct. */
  acq_state.state = ACQ_RUNNING;
  acq_state.prn = prn;
  acq_state.best_power = 0;
  acq_state.power_acc = 0;
  acq_state.count = 0;
  acq_state.carrier_freq = acq_state.cf_min;
  acq_state.code_phase = acq_state.cp_min;

  /* Write first and second sets of acq parameters (for pipelining). */
  nap_acq_init_wr_params_blocking(prn, acq_state.cp_min, acq_state.cf_min);
  /* TODO: If we are only doing a single acq then write disable here. */
  nap_acq_init_wr_params_blocking(prn, acq_state.cp_min+nap_acq_n_taps, acq_state.cf_min);
}

/** Handle an acquisition done interrupt from the NAP acquisition channel.
 * If acq_state.state =
 *   ACQ_RUNNING :
 *     write the next set of pipelined acquisition parameters.
 *   ACQ_RUNNING_FINISHING :
 *     channel is currently doing the final acquisition of the set of
 *     acquisitions, write a pipelined disable to stop the channel after this
 *     final acquisition.
 */
void acq_service_irq()
{
  u64 power;
  corr_t cs[nap_acq_n_taps];

  switch(acq_state.state)
  {
    default:
      /* If we get an interrupt when we are not running,
       * disable the acq channel which helpfully also
       * clears the IRQ.
       */
      printf("!!! Acq state error? %d\n", acq_state.state);
      nap_acq_init_wr_disable_blocking();
      break;

    case ACQ_RUNNING_FINISHING:
      nap_acq_init_wr_disable_blocking();
      acq_state.state = ACQ_RUNNING_DONE;
      break;

    case ACQ_RUNNING:
      /* Read in correlations. */
      nap_acq_corr_rd_blocking(cs);

      /* Write parameters for 2 cycles time for acq pipelining apart
       * from the last two cycles where we want to write disable.
       * The first time to disable and the second time really just
       * to clear the interrupt from the last cycle.
       *
       * NOTE: we must take care to handle wrapping, when we get to
       * the end of the code phase range the parameters for 2 cycles
       * time will be with the next carrier freq value and a small
       * code phase value.
       */
      if (acq_state.code_phase < acq_state.cp_max - 2*nap_acq_n_taps) {
        nap_acq_init_wr_params_blocking(acq_state.prn, \
          acq_state.code_phase+2*nap_acq_n_taps, \
          acq_state.carrier_freq);
      } else {
        if (acq_state.carrier_freq >= acq_state.cf_max && \
            acq_state.code_phase >= (acq_state.cp_max-2*nap_acq_n_taps)) {
          nap_acq_init_wr_disable_blocking();
          acq_state.state = ACQ_RUNNING_FINISHING;
        } else {
          nap_acq_init_wr_params_blocking(acq_state.prn, \
            acq_state.cp_min + acq_state.code_phase - acq_state.cp_max + 2*nap_acq_n_taps, \
            acq_state.carrier_freq+acq_state.cf_step);
        }
      }

      for (u8 i=0; i<nap_acq_n_taps; i++) {
        power = (u64)cs[i].I*(u64)cs[i].I + (u64)cs[i].Q*(u64)cs[i].Q;
        acq_state.power_acc += power;
        if (power > acq_state.best_power) {
          acq_state.best_power = power;
          acq_state.best_cf = acq_state.carrier_freq;
          acq_state.best_cp = acq_state.code_phase + i;
        }
      }
      acq_state.count += nap_acq_n_taps;
      acq_state.code_phase += nap_acq_n_taps;
      if (acq_state.code_phase >= acq_state.cp_max) {
        acq_state.code_phase = acq_state.cp_min;
        acq_state.carrier_freq += acq_state.cf_step;
      }
      break;
  }
}

/** Query the state of the acquisition sequence.
 * \return 1 if acq_state = ACQ_RUNNING_DONE, 0 otherwise
 */
u8 acq_get_done()
{
  return (acq_state.state == ACQ_RUNNING_DONE);
}

/** Get the results of the set of acquisitions last performed.
 * Get the code phase, carrier frequency, and SNR of the acquisition with the
 * highest SNR of set of acquisitions last performed.
 *
 * \param cp  Code phase of the acquisition result
 * \param cf  Carrier frequency of the acquisition result
 * \param snr SNR of the acquisition result
 */
void acq_get_results(float* cp, float* cf, float* snr)
{
  *cp = (float)acq_state.best_cp / NAP_ACQ_CODE_PHASE_UNITS_PER_CHIP;
  *cf = (float)acq_state.best_cf / NAP_ACQ_CARRIER_FREQ_UNITS_PER_HZ;
  /* "SNR" estimated by peak power over mean power. */
  *snr = (float)acq_state.best_power / (acq_state.power_acc / acq_state.count);
}

/** Do a blocking acquisition in two stages : coarse and fine.
 * Do a coarse acqusition to find the approximate code phase and carrier
 * frequency, and then a more fine grained acquisition to find the code phase
 * and carrier frequency more precisely.
 *
 * \param prn PRN to search (nap_acq_code_wr_blocking must be called prior)
 * \param cp  Code phase of the acquisition result
 * \param cf  Carrier frequency of the acquisition result
 * \param snr SNR of the acquisition result
 */
u32 acq_full_two_stage(u8 prn, float* cp, float* cf, float* snr)
{
  /* Initial coarse acq. */
  float coarse_code_phase;
  float coarse_carrier_freq;
  float coarse_snr;

  u32 coarse_count = nap_timing_count() + 1000;
  acq_schedule_load(coarse_count);
  while(!acq_get_load_done());

  acq_start(prn, 0, 1023, -7000, 7000, 300);
  while(!acq_get_done());
  acq_get_results(&coarse_code_phase, &coarse_carrier_freq, &coarse_snr);

  /* Fine acq. */
  u32 fine_count = nap_timing_count() + 2000;
  acq_schedule_load(fine_count);
  while(!acq_get_load_done());

  float fine_cp = propagate_code_phase(coarse_code_phase, coarse_carrier_freq, fine_count - coarse_count);

  acq_start(prn, fine_cp-20, fine_cp+20, coarse_carrier_freq-300, coarse_carrier_freq+300, 100);
  while(!acq_get_done());
  acq_get_results(cp, cf, snr);

  return fine_count;
}

/** Perform an aqcuisition.
 * Perform an acquisition for one PRN over a defined code and doppler range.
 * Returns the code phase and carrier frequency of the largest peak in the
 * search space together with the "SNR" value for that peak defined as
 * (peak_magnitude - mean) / std_deviation.
 *
 * \param prn    PRN number - 1 (0..31) to attempt to acquire
 *               (nap_acq_code_wr_blocking must be called prior).
 * \param cp_min Lower bound for code phase search range in chips.
 * \param cp_max Upper bound for code phase search range in chips.
 * \param cf_min Lower bound for carrier freq. search range in Hz.
 * \param cf_max Upper bound for carrier freq. search range in Hz.
 * \param cp     Pointer to a float where the peak's code phase value will be
 *               stored in chips.
 * \param cf     Pointer to a float where the peak's carrier frequency will be
 *               stored in Hz.
 * \param snr    Pointer to a float where the "SNR" of the peak will be stored.
 */
void do_acq(u8 prn, float cp_min, float cp_max, float cf_min, float cf_max, float cf_bin_width, float* cp, float* cf, float* snr)
{
  acq_start(prn, cp_min, cp_max, cf_min, cf_max, cf_bin_width);
  while(acq_state.state == ACQ_RUNNING) {
    wait_for_nap_exti();
    acq_service_irq();
  }
  wait_for_nap_exti();
  acq_service_irq();
  acq_get_results(cp, cf, snr);
}

/** \} */

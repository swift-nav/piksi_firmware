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
#include <libopencm3/stm32/f2/gpio.h>

#include "swift_nap_io.h"

u8 acq_load_done_flag = 0;

void do_one_acq(u8 prn, u16 code_phase, s16 carrier_freq, corr_t corrs[])
{
  acq_write_init_blocking(prn, code_phase, carrier_freq); 

  /* Disable acq on next cycle, after this one has finished. */
  acq_disable_blocking();

  /* Wait for acq done IRQ. */
  wait_for_exti();

  /* Write to clear IRQ. */
  acq_disable_blocking();

  /* Read in correlations. */
  acq_read_corr_blocking(corrs);
}

void acq_schedule_load(u32 count)
{
  acq_load_done_flag = 0;
  acq_set_load_enable_blocking();
  timing_strobe(count);
}

void acq_service_load_done()
{
  acq_clear_load_enable_blocking();
  acq_load_done_flag = 1;
}

void acq_wait_load_done()
{
  while(acq_load_done_flag != 1);
  acq_load_done_flag = 0;
}

/** Perform an aqcuisition.
 * Perform an acquisition for one PRN over a defined code and doppler range. Returns
 * the code phase and carrier frequency of the largest peak in the search space together
 * with the "SNR" value for that peak defined as (peak_magnitude - mean) / std_deviation.
 *
 * \param prn    PRN number - 1 (0..31) to attempt to aqcuire.
 * \param cp_min Lower bound for code phase search range in chips.
 * \param cp_max Upper bound for code phase search range in chips.
 * \param cf_min Lower bound for carrier freq. search range in Hz.
 * \param cf_max Upper bound for carrier freq. search range in Hz.
 * \param cp     Pointer to a float where the peak's code phase value will be stored in chips.
 * \param cf     Pointer to a float where the peak's carrier frequency will be stored in Hz.
 * \param snr    Pointer to a float where the "SNR" of the peak will be stored.
 */
void do_acq(u8 prn, float cp_min, float cp_max, float cf_min, float cf_max, float cf_bin_width, float* cp, float* cf, float* snr)
{
  corr_t cs[ACQ_N_TAPS];

  float mag, mag_sq, best_mag = 0;
  float sd, mean = 0, sq_mean = 0;
  u32 count = 0;
  u16 best_cp = 0;
  s16 best_cf = 0;

  /* Calculate the range parameters in acq units. Explicitly expand
   * the range to the nearest multiple of the step size to make sure
   * we cover at least the specified range.
   */
  s16 cf_step = cf_bin_width*ACQ_CARRIER_FREQ_UNITS_PER_HZ;
  s16 cf_min_acq = cf_step*floor(cf_min*ACQ_CARRIER_FREQ_UNITS_PER_HZ / (float)cf_step);
  s16 cf_max_acq = cf_step*ceil(cf_max*ACQ_CARRIER_FREQ_UNITS_PER_HZ / (float)cf_step);
  /* cp_step = ACQ_N_TAPS */
  u16 cp_min_acq = ACQ_N_TAPS*floor(cp_min*ACQ_CODE_PHASE_UNITS_PER_CHIP / (float)ACQ_N_TAPS);
  u16 cp_max_acq = ACQ_N_TAPS*ceil(cp_max*ACQ_CODE_PHASE_UNITS_PER_CHIP / (float)ACQ_N_TAPS);

  /* Write first and second sets of acq parameters (for pipelining). */
  acq_write_init_blocking(prn, cp_min_acq, cf_min_acq); 
  /* If we are only doing a single acq then write disable here. */
  /*printf("(%d, %d) => \n", cp_min_acq+ACQ_N_TAPS, cf_min_acq);*/
  /*if (cp_min_acq+ACQ_N_TAPS >= cp_max_acq && cf_min_acq+cf_step >= cf_max_acq) {*/
          /*printf(" D\n");*/
    /*acq_disable();*/
  /*}else*/
    acq_write_init_blocking(prn, cp_min_acq+ACQ_N_TAPS, cf_min_acq); 

  /* Save the exti count so we can detect the next interrupt. */
  u32 last_last_exti = last_exti_count();

  for (s16 carrier_freq = cf_min_acq; carrier_freq <= cf_max_acq; carrier_freq += cf_step) {
    for (u16 code_phase = cp_min_acq; code_phase < cp_max_acq; code_phase += ACQ_N_TAPS) {

      /* Wait for acq done IRQ and save the new exti count. */
      while(last_exti_count() == last_last_exti);
      last_last_exti = last_exti_count();

      /* Read in correlations. */
      acq_read_corr_blocking(cs);

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
      if (code_phase < cp_max_acq - 2*ACQ_N_TAPS) {
        /*printf("(%d, %d) => %d, %d\n", code_phase, carrier_freq, code_phase+2*ACQ_N_TAPS, carrier_freq);*/
        acq_write_init_blocking(prn, code_phase+2*ACQ_N_TAPS, carrier_freq); 
      } else {
        if (carrier_freq >= cf_max_acq && code_phase >= (cp_max_acq-2*ACQ_N_TAPS)) {
          /*printf("(%d, %d) => D\n", code_phase, carrier_freq);*/
          acq_disable_blocking();
        } else {
          /*printf("(%d, %d) => %d, %d\n", code_phase, carrier_freq, cp_min_acq+code_phase-cp_max_acq+2*ACQ_N_TAPS, carrier_freq+cf_step);*/
          acq_write_init_blocking(prn, cp_min_acq + code_phase - cp_max_acq + 2*ACQ_N_TAPS, carrier_freq+cf_step); 
        }
      }

      for (u8 i=0; i<ACQ_N_TAPS; i++) {
        mag_sq = (float)cs[i].I*(float)cs[i].I + (float)cs[i].Q*(float)cs[i].Q;
        mag = sqrt(mag_sq);
        mean += mag;
        sq_mean += mag_sq;
        if (mag > best_mag) {
          best_mag = mag;
          best_cf = carrier_freq;
          best_cp = code_phase + i;
        }
        /*printf("%f, %f, %f, %f, %f, %d, %d\n", mag_sq, mag, mean, sq_mean, best_mag, best_cf, best_cp);*/
      }
      count += ACQ_N_TAPS;
    }
  }

  sd = sqrt(count*sq_mean - mean*mean) / count;
  mean = mean / count;

  *cp = (float)best_cp / ACQ_CODE_PHASE_UNITS_PER_CHIP;
  *cf = (float)best_cf / ACQ_CARRIER_FREQ_UNITS_PER_HZ;
  *snr = (best_mag - mean) / sd;
}


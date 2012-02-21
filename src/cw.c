/*
 * Copyright (C) 2012 Colin Beighley <colinbeighley@gmail.com>
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

#include "swift_nap_io.h"
#include "cw.h"

cw_state_t cw_state;

void do_one_cw(s32 carrier_freq, corr_t* corrs)
{
  cw_write_init_blocking(carrier_freq); 

  /* Disable cw on next cycle, after this one has finished. */
  cw_disable_blocking();

  /* Wait for cw done IRQ. */
  wait_for_exti();

  /* Write to clear IRQ. */
  cw_disable_blocking();

  /* Read in correlations. */
  cw_read_corr_blocking(corrs);
}

void cw_schedule_load(u32 count)
{
  cw_state.state = CW_LOADING;
  cw_set_load_enable_blocking();
  timing_strobe(count);
}

void cw_service_load_done()
{
  cw_clear_load_enable_blocking();
  cw_state.state = CW_LOADING_DONE;
}

u8 cw_get_load_done()
{
  return (cw_state.state == CW_LOADING_DONE);
}

u8 cw_get_running_done()
{
  return (cw_state.state == CW_RUNNING_DONE);
}

void cw_start(float cf_min, float cf_max, float cf_bin_width)
{
  /* Calculate the range parameters in cw units. Explicitly expand
   * the range to the nearest multiple of the step size to make sure
   * we cover at least the specified range.
   */
//  cw_state.cf_step = cf_bin_width*CW_CARRIER_FREQ_UNITS_PER_HZ;
//  cw_state.cf_min = cw_state.cf_step*floor(cf_min*CW_CARRIER_FREQ_UNITS_PER_HZ / (float)cw_state.cf_step);
//  cw_state.cf_max = cw_state.cf_step*ceil(cf_max*CW_CARRIER_FREQ_UNITS_PER_HZ / (float)cw_state.cf_step);
  cw_state.cf_step = ceil(cf_bin_width*CW_CARRIER_FREQ_UNITS_PER_HZ);
  cw_state.cf_min = cf_min*CW_CARRIER_FREQ_UNITS_PER_HZ;
  cw_state.cf_max = cf_max*CW_CARRIER_FREQ_UNITS_PER_HZ;
//	printf("%d %d %d\n",(int)cw_state.cf_min,(int)cw_state.cf_max,(int)cw_state.cf_step);

  /* Initialise our cw state struct. */
  cw_state.state = CW_RUNNING;
  cw_state.power_acc = 0;
  cw_state.best_power = 0;
  cw_state.best_freq = 0;
  cw_state.count = 0;
  cw_state.carrier_freq = cw_state.cf_min;

  /* Write first and second sets of detection parameters (for pipelining). */
  cw_write_init_blocking(cw_state.cf_min);
  /* TODO: If we are only doing a single detection then write disable here. */
  cw_write_init_blocking(cw_state.carrier_freq + cw_state.cf_step);
}

void cw_service_irq()
{
  u64 power;
  corr_t cs;

  switch(cw_state.state)
  {
    default:
      /* If we get an interrupt when we are not running,
       * disable the cw channel which helpfully also
       * clears the IRQ.
       */
      cw_disable_blocking();
      break;

    case CW_RUNNING:
      /* Read in correlations. */
      cw_read_corr_blocking(&cs);

      power = (u64)cs.I*(u64)cs.I + (u64)cs.Q*(u64)cs.Q;
      cw_state.power_acc += power;
      if (power > cw_state.best_power) {
        cw_state.best_power = power;
        cw_state.best_freq = cw_state.carrier_freq;
      }

//			printf("%d %d %d\n",(unsigned int)power,(unsigned int)cw_state.carrier_freq,(unsigned int)cw_state.count);

      if (cw_state.count < SPECTRUM_LEN) {
        cw_state.spectrum_power[cw_state.count] = power;
        cw_state.spectrum_freq[cw_state.count] = cw_state.carrier_freq;
      }
      cw_state.count++;

      /* Write parameters for 2 cycles time for cw pipelining apart
       * from the last two cycles where we want to write disable.
       * The first time to disable and the second time really just
       * to clear the interrupt from the last cycle.
       */
      cw_state.carrier_freq += cw_state.cf_step; //carrier freq corresponding to correlations 
																								 //that will be read from channel next cycle
																								 //(unless we are writing second disable)
			if (cw_state.carrier_freq >= (cw_state.cf_max + cw_state.cf_step)) { //second disable, lower cw channel
        cw_disable_blocking();                                             //interrupt and transition state
        cw_state.state = CW_RUNNING_DONE;
			} else if (cw_state.carrier_freq >= cw_state.cf_max) { //first disable, disable cw channel
        cw_disable_blocking();
			} else {
        cw_write_init_blocking(cw_state.carrier_freq + cw_state.cf_step);
			}

      break;
  }
}

void cw_get_results(float* cf, float* snr)
{
  *cf = (float)cw_state.best_freq / CW_CARRIER_FREQ_UNITS_PER_HZ;
  /* "SNR" estimated by peak power over mean power. */
  *snr = (float)cw_state.best_power / (cw_state.power_acc / cw_state.count);
}

void cw_get_spectrum_point(float* freq, u64* power, u16 index)
//void cw_get_spectrum_point(float* freq, float* power, u16 index)
{
	*freq = (float)cw_state.spectrum_freq[index] / CW_CARRIER_FREQ_UNITS_PER_HZ;
	*power = cw_state.spectrum_power[index];
//	*power = (float)cw_state.spectrum_power[index];
}


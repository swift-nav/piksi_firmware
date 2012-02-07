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

#ifndef SWIFTNAV_ACQ_H
#define SWIFTNAV_ACQ_H

#include <libopencm3/cm3/common.h>

#define ACQ_THRESHOLD 16.0

/* NOTE: Minimum bin width 1/ACQ_CARRIER_FREQ_UNITS_PER_HZ (~16 Hz) */
#define ACQ_CARRIER_BIN_WIDTH 300

typedef enum {
  ACQ_DISABLED = 0,
  ACQ_RUNNING,
  ACQ_DONE
} acq_status_t;

typedef struct {
  acq_status_t state;
  u8 prn;
  s16 cf_step, cf_min, cf_max;
  u16 cp_min, cp_max;
  s16 carrier_freq;
  u16 code_phase;
  float mean, sq_mean;
  float best_mag;
  s16 best_cf;
  u16 best_cp;
  u32 count;
} acq_state_t;

void acq_schedule_load(u32 count);
void acq_service_load_done();
void acq_wait_load_done();

void acq_start(u8 prn, float cp_min, float cp_max, float cf_min, float cf_max, float cf_bin_width);
void acq_service_irq();
void acq_wait_done();
void acq_get_results(float* cp, float* cf, float* snr);

u32 acq_full_two_stage(u8 prn, float* cp, float* cf, float* snr);

void do_one_acq(u8 prn, u16 code_phase, s16 carrier_freq, corr_t corrs[]);
void do_acq(u8 prn, float cp_min, float cp_max, float cf_min, float cf_max, float cf_bin_width, float* cp, float* cf, float* snr);

#endif

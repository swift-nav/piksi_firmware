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

/** \addtogroup acq
 * \{ */

/** Status of SwiftNAP acquisition channel. */
typedef enum {
  ACQ_DISABLED = 0,
  ACQ_LOADING,
  ACQ_LOADING_DONE,
  ACQ_RUNNING,
  ACQ_RUNNING_FINISHING,
  ACQ_RUNNING_DONE
} acq_status_t;

/** Acquisition search state. */
typedef struct {
  acq_status_t state; /**< Status of SwiftNAP acquisition channel. */
  u8 prn;             /**< C/A Code (0-31) being searched for. */
  s16 cf_step;        /**< Step size between carrier freq search points. */
  s16 cf_min;         /**< Lowest carrier freq to search. */
  s16 cf_max;         /**< Highest carrier freq to search. */
  u16 cp_min;         /**< Lowest code phase to search. */
  u16 cp_max;         /**< Highest code phase to search. */
  s16 carrier_freq;   /**< Carrier freq of next correlation to be read. */
  u16 code_phase;     /**< Code phase of next correlation to be read. */
  u64 power_acc;      /**< Sum of powers of all acquisition set points. */
  u64 best_power;     /**< Highest power of all acquisition set points. */
  s16 best_cf;        /**< Carrier freq corresponding to highest power. */
  u16 best_cp;        /**< Code phase corresponding to highest power. */
  u32 count;          /**< Total number of acquisition points searched. */
} acq_state_t;

/** \} */

void acq_schedule_load(u32 count);
void acq_service_load_done();
u8 acq_get_load_done();

void acq_start(u8 prn, float cp_min, float cp_max, float cf_min, float cf_max, float cf_bin_width);
void acq_service_irq();
u8 acq_get_done();
void acq_get_results(float* cp, float* cf, float* snr);

u32 acq_full_two_stage(u8 prn, float* cp, float* cf, float* snr);

void do_acq(u8 prn, float cp_min, float cp_max, float cf_min, float cf_max, float cf_bin_width, float* cp, float* cf, float* snr);

#endif

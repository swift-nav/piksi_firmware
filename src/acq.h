/*
 * Copyright (C) 2011-2013 Swift Navigation Inc.
 * Contact: Fergus Noble <fnoble@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
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

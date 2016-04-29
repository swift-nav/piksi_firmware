/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_NAP_COMMON_H
#define SWIFTNAV_NAP_COMMON_H

#include <ch.h>

#include <libswiftnav/common.h>
#include "../../ext_events.h"

/** \addtogroup nap
 * \{ */

/** A complex IQ correlation. */
typedef struct {
  s32 I;  /**< In-phase correlation. */
  s32 Q;  /**< Quadrature correlation. */
} corr_t;

/** Accumulation of IQ correlations.
 * Used to receive accumulation of correlations from Acq channel in the form
 * acc.I = sum(tap[N].corr_i^2) for N = 0 to NAP_ACQ_N_TAPS-1
 * acc.Q = sum(tap[N].corr_q^2) for N = 0 to NAP_ACQ_N_TAPS-1
 */
typedef struct {
  u64 I;  /**< In-phase correlation accumulation. */
  u64 Q;  /**< Quadrature correlation accumulation. */
} acc_t;

extern binary_semaphore_t timing_strobe_sem;

/** \} */

void nap_setup(void);

u32 nap_error_rd_blocking(void);

u8 nap_hash_status(void);

void nap_rd_dna(u8 dna[]);

void nap_callbacks_setup(void);

u64 nap_timing_count(void);
u32 nap_timing_count_latched(void);
void nap_timing_strobe(u32 falling_edge_count);
bool nap_timing_strobe_wait(u32 timeout);

u32 nap_rw_ext_event(u8 *event_pin, ext_event_trigger_t *event_trig,
		     ext_event_trigger_t next_trig);

void nap_pps(u64 rising_edge_count_8x);
void nap_pps_width(u32 falling_edge_count);

#include "nap/nap_hw.h"

#endif  /* SWIFTNAV_NAP_COMMON_H */


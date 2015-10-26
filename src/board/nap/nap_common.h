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

#include <libswiftnav/common.h>
#include "../../ext_events.h"

/** \addtogroup nap
 * \{ */

/* NAP Register Addresses. */
#define NAP_REG_IRQ                 0x00
#define NAP_REG_ERROR               0x01
#define NAP_REG_PPS_WIDTH           0xF4
#define NAP_REG_PPS_COMPARE         0xF5
#define NAP_REG_EXT_EVENT_TIME      0xF6
#define NAP_REG_TIMING_COMPARE      0xF8
#define NAP_REG_TIMING_COUNT        0xF9
#define NAP_REG_TIMING_COUNT_LATCH  0xFA
#define NAP_REG_HASH_STATUS         0xFB
#define NAP_REG_DNA                 0xFD
#define NAP_REG_RDWR                0xFE
#define NAP_REG_DECEASED_COW        0xFF
/* Status of NAP authentication hash comparison. */
/* TODO: change NAP_HASH_MATCH to non 0x00/0xFF value so it is more reliable */
#define NAP_HASH_MATCH              0
#define NAP_HASH_MISMATCH           1
#define NAP_HASH_NOTREADY           2

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

/** \} */

void nap_setup(void);

u8 nap_conf_done(void);
u8 nap_hash_rd_done(void);

void nap_conf_b_setup(void);
void nap_conf_b_set(void);
void nap_conf_b_clear(void);

void nap_xfer_blocking(u8 reg_id, u16 n_bytes, u8 data_in[],
                       const u8 data_out[]);

/** Convenience function to read 4 bytes from a register (writing zeros) and
 * convert to host byte order (i.e. little-endian).
 * \param reg_id   NAP register ID.
 * \return u32 register value
 */
inline u32 nap_read_u32(u8 reg_id)
{
  u32 val = 0;
  nap_xfer_blocking(reg_id, 4, (u8 *)&val, (u8 *)&val);
  return __builtin_bswap32(val);
}

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

#endif  /* SWIFTNAV_NAP_COMMON_H */


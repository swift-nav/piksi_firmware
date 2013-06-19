/*
 * Copyright (C) 2011-2013 Swift Navigation Inc.
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

/** \addtogroup nap
 * \{ */

/* NAP Register Addresses. */
#define NAP_REG_IRQ                 0x00
#define NAP_REG_ERROR               0x01
#define NAP_REG_TIMING_COMPARE      0xF8
#define NAP_REG_TIMING_COUNT        0xF9
#define NAP_REG_TIMING_COUNT_LATCH  0xFA
#define NAP_REG_HASH_STATUS         0xFB
#define NAP_REG_DNA                 0xFD
#define NAP_REG_RDWR                0xFE
#define NAP_REG_DECEASED_COW        0xFF

/* Status of NAP authentication hash comparison. */
#define NAP_HASH_MATCH              0
#define NAP_HASH_MISMATCH           1
#define NAP_HASH_NOTREADY           2

/* NAP IRQ register bit definitions. */
#define NAP_IRQ_ACQ_DONE            (1 << 31)
#define NAP_IRQ_ACQ_LOAD_DONE       (1 << 30)
#define NAP_IRQ_CW_DONE             (1 << 29)
#define NAP_IRQ_CW_LOAD_DONE        (1 << 28)
#define NAP_IRQ_TRACK_MASK          (~(NAP_IRQ_ACQ_DONE | \
                                       NAP_IRQ_ACQ_LOAD_DONE | \
                                       NAP_IRQ_CW_DONE | \
                                       NAP_IRQ_CW_LOAD_DONE))

/** Structure containing a complex IQ correlation. */
typedef struct {
  s32 I;  /**< In-phase correlation. */
  s32 Q;  /**< Quadrature correlation. */
} corr_t;

/** \} */

void nap_setup(void);
void nap_reset(void);

u8 nap_conf_done(void);
u8 nap_hash_rd_done(void);

void nap_conf_b_setup(void);
void nap_conf_b_set(void);
void nap_conf_b_clear(void);

void nap_exti_setup(void);
u32 last_nap_exti_count(void);
void wait_for_nap_exti(void);

void nap_xfer_blocking(u8 reg_id, u16 n_bytes, u8 data_in[],
                       const u8 data_out[]);

u32 nap_irq_rd_blocking(void);
u32 nap_error_rd_blocking(void);

u8 nap_hash_status(void);

void nap_rd_dna(u8 dna[]);
void nap_rd_dna_callback(u8 buff[]);

void nap_callbacks_setup(void);

u64 nap_timing_count(void);
u32 nap_timing_count_latched(void);
void nap_timing_strobe(u32 falling_edge_count);

#endif  /* SWIFTNAV_NAP_COMMON_H */


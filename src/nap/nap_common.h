/*
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
 * Copyright (C) 2013 Colin Beighley <colinbeighley@gmail.com>
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

#ifndef SWIFTNAV_NAP_COMMON_H
#define SWIFTNAV_NAP_COMMON_H

#include <libopencm3/cm3/common.h>

/* NAP Register Addresses. */
#define NAP_REG_IRQ                0x00
#define NAP_REG_ERROR              0x01
#define NAP_REG_TIMING_COMPARE     0xF8
#define NAP_REG_TIMING_COUNT       0xF9
#define NAP_REG_TIMING_COUNT_LATCH 0xFA
#define NAP_REG_HASH_STATUS        0xFB
#define NAP_REG_DNA                0xFD
#define NAP_REG_RDWR               0xFE
#define NAP_REG_DECEASED_COW       0xFF

/* Status of NAP authentication hash comparison. */
#define NAP_HASH_MATCH    0
#define NAP_HASH_MISMATCH 1
#define NAP_HASH_NOTREADY 2

/* NAP IRQ register bit definitions. */
#define NAP_IRQ_ACQ_DONE      (1<<31)
#define NAP_IRQ_ACQ_LOAD_DONE (1<<30)
#define NAP_IRQ_CW_DONE       (1<<29)
#define NAP_IRQ_CW_LOAD_DONE  (1<<28)
#define NAP_IRQ_TRACK_MASK    (~(NAP_IRQ_ACQ_DONE|NAP_IRQ_ACQ_LOAD_DONE|NAP_IRQ_CW_DONE|NAP_IRQ_CW_LOAD_DONE))

typedef struct {
  s32 I;
  s32 Q;
} corr_t;

void nap_setup();
void nap_reset();

u8 nap_conf_done();
u8 nap_hash_rd_done();

void nap_conf_b_setup();
void nap_conf_b_set();
void nap_conf_b_clear();

void nap_exti_setup();
u32 nap_last_exti_count();
void nap_wait_for_exti();

void nap_xfer_blocking(u8 reg_id, u16 n_bytes, u8 data_in[], const u8 data_out[]);

u32 nap_read_irq_blocking();
u32 nap_read_error_blocking();

u8 nap_hash_status();

void get_nap_dna(u8 dna[]);
void get_nap_dna_callback();

void nap_callbacks_setup();

u64 nap_timing_count();
u32 nap_timing_count_latched();
void nap_timing_strobe(u32 falling_edge_count);

#endif /* SWIFTNAV_NAP_COMMON_H */

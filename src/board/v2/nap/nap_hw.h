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

#ifndef SWIFTNAV_NAP_HW_V2_H
#define SWIFTNAV_NAP_HW_V2_H

#include <libswiftnav/signal.h>

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

/** Max number of tracking channels NAP configuration will be built with. */
#define NAP_MAX_N_TRACK_CHANNELS    (MAX_CHANNELS + 1)

u8 nap_conf_done(void);
u8 nap_hash_rd_done(void);

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

void nap_track_code_wr_blocking(u8 channel, gnss_signal_t sid);
void nap_track_init_wr_blocking(u8 channel, u8 prn, s32 carrier_phase,
                                u16 code_phase);
void nap_track_update_wr_blocking(u8 channel, s32 carrier_freq,
                                  u32 code_phase_rate, u8 rollover_count,
                                  u8 corr_spacing);
void nap_track_corr_rd_blocking(u8 channel, u32* sample_count, corr_t corrs[]);

#endif


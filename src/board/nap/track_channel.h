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

#ifndef SWIFTNAV_TRACK_CHANNEL_H
#define SWIFTNAV_TRACK_CHANNEL_H

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>

#include "../../main.h"
#include "nap_common.h"

/** \addtogroup track_channel
 * \{ */

/* SPI register IDs */
#define NAP_REG_TRACK_BASE           0x0A
#define NAP_TRACK_N_REGS             5
#define NAP_REG_TRACK_INIT_OFFSET    0x00
#define NAP_REG_TRACK_UPDATE_OFFSET  0x01
#define NAP_REG_TRACK_CORR_OFFSET    0x02
#define NAP_REG_TRACK_PHASE_OFFSET   0x03
#define NAP_REG_TRACK_CODE_OFFSET    0x04

/** Max number of tracking channels NAP configuration will be built with. */
#define NAP_MAX_N_TRACK_CHANNELS     12

extern u8 nap_track_n_channels;

/* NAP track channel parameters.
 * TODO : get rid of some of these INIT reg specific defines by just writing
 * whole phase through init register? Tracking channel init registers don't
 * get written very often so it shouldn't increase SPI link budget much.
 */
#define NAP_TRACK_INIT_CODE_PHASE_WIDTH           14
#define NAP_TRACK_CARRIER_FREQ_WIDTH              24
#define NAP_TRACK_CODE_PHASE_WIDTH                29
#define NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH     32

#define NAP_TRACK_INIT_CODE_PHASE_UNITS_PER_CHIP  \
  (1 << (NAP_TRACK_INIT_CODE_PHASE_WIDTH - 10))

#define NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ       \
  ((1 << NAP_TRACK_CARRIER_FREQ_WIDTH) / (double)SAMPLE_FREQ)

#define NAP_TRACK_NOMINAL_CODE_PHASE_RATE         \
  (1 << (NAP_TRACK_CODE_PHASE_WIDTH - 1))

#define NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ    \
  (NAP_TRACK_NOMINAL_CODE_PHASE_RATE / 1.023e6)

#define NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP       \
  ((u64)1 << NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH)

/** \} */

void nap_track_init_pack(u8 pack[], u8 prn, s32 carrier_phase, u16 code_phase);
void nap_track_init_wr_blocking(u8 channel, u8 prn, s32 carrier_phase,
                                u16 code_phase);
void nap_track_update_pack(u8 pack[], s32 carrier_freq, u32 code_phase_rate,
                           u8 rollover_count, u8 corr_spacing);
void nap_track_update_wr_blocking(u8 channel, s32 carrier_freq,
                                  u32 code_phase_rate, u8 rollover_count,
                                  u8 corr_spacing);
void nap_track_corr_unpack(u8 packed[], u32* sample_count, corr_t corrs[]);
void nap_track_corr_rd_blocking(u8 channel, u32* sample_count, corr_t corrs[]);
void nap_track_phase_unpack(u8 packed[], s32* carrier_phase, u64* code_phase);
void nap_track_phase_rd_blocking(u8 channel, s32* carrier_phase,
                                 u64* code_phase);
void nap_track_code_wr_blocking(u8 channel, gnss_signal_t sid);

#endif  /* SWIFTNAV_TRACK_CHANNEL_H */


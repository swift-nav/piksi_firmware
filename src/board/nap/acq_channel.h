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

#ifndef SWIFTNAV_ACQ_CHANNEL_H
#define SWIFTNAV_ACQ_CHANNEL_H

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>

#include "../../main.h"
#include "nap_common.h"

/** \addtogroup acq_channel
 * \{ */

#define NAP_REG_ACQ_BASE 0x06
#define NAP_REG_ACQ_INIT (NAP_REG_ACQ_BASE + 0x00)
#define NAP_REG_ACQ_LOAD (NAP_REG_ACQ_BASE + 0x01)
#define NAP_REG_ACQ_CORR (NAP_REG_ACQ_BASE + 0x02)
#define NAP_REG_ACQ_CODE (NAP_REG_ACQ_BASE + 0x03)

#define NAP_ACQ_PIPELINE_STAGES           1
#define NAP_ACQ_CODE_PHASE_WIDTH          11
#define NAP_ACQ_CARRIER_FREQ_WIDTH        nap_acq_fft_index_bits
#define NAP_ACQ_SAMPLE_FREQ               (SAMPLE_FREQ/(4*nap_acq_downsample_stages))
#define NAP_ACQ_CODE_PHASE_UNITS_PER_CHIP (NAP_ACQ_SAMPLE_FREQ/1023000)
#define NAP_ACQ_CARRIER_FREQ_UNITS_PER_HZ ((1 << NAP_ACQ_CARRIER_FREQ_WIDTH) \
                                           / (float)NAP_ACQ_SAMPLE_FREQ)

/** \} */

/** Number of acq. channel code phase taps.
 * Number of acquisition channel code phase taps that NAP configuration was
 * built with. Read from configuration flash at runtime in
 * nap_conf_rd_parameters().
 */
extern u8 nap_acq_fft_index_bits;
extern u8 nap_acq_downsample_stages;

void nap_acq_load_wr_enable_blocking(void);
void nap_acq_load_wr_disable_blocking(void);
void nap_acq_init_wr_params_blocking(s16 carrier_freq);
void nap_acq_init_wr_disable_blocking(void);
void nap_acq_corr_rd_blocking(u16 *index, u16 *max, float *ave);
void nap_acq_code_wr_blocking(gnss_signal_t sid);

#endif  /* SWIFTNAV_ACQ_CHANNEL_H */


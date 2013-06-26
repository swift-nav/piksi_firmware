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

#ifndef SWIFTNAV_CW_CHANNEL_H
#define SWIFTNAV_CW_CHANNEL_H

#include <libswiftnav/common.h>

#include "../../main.h"
#include "nap_common.h"

/** \addtogroup cw_channel
 * \{ */

#define NAP_REG_CW_BASE 0x02
#define NAP_REG_CW_INIT (NAP_REG_CW_BASE + 0x00)
#define NAP_REG_CW_LOAD (NAP_REG_CW_BASE + 0x01)
#define NAP_REG_CW_CORR (NAP_REG_CW_BASE + 0x02)

#define NAP_CW_FREQ_WIDTH         20
#define NAP_CW_FREQ_UNITS_PER_HZ  ((1 << NAP_CW_FREQ_WIDTH) \
                                   / (float)SAMPLE_FREQ)

/** \} */

void nap_cw_load_wr_enable_blocking(void);
void nap_cw_load_wr_disable_blocking(void);
void nap_cw_init_pack(u8 pack[], s32 carrier_freq);
void nap_cw_init_wr_params_blocking(s32 carrier_freq);
void nap_cw_init_wr_disable_blocking(void);
void nap_cw_corr_unpack(u8 packed[], corr_t* corrs);
void nap_cw_corr_rd_blocking(corr_t* corrs);

#endif  /* SWIFTNAV_CW_CHANNEL_H */


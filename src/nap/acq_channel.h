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

#ifndef SWIFTNAV_ACQ_CHANNEL_H
#define SWIFTNAV_ACQ_CHANNEL_H

#include <libopencm3/cm3/common.h>

#include "nap_common.h"
#include "../main.h"

#define NAP_REG_ACQ_BASE 0x06
#define NAP_REG_ACQ_INIT (NAP_REG_ACQ_BASE+0x00)
#define NAP_REG_ACQ_LOAD (NAP_REG_ACQ_BASE+0x01)
#define NAP_REG_ACQ_CORR (NAP_REG_ACQ_BASE+0x02)
#define NAP_REG_ACQ_CODE (NAP_REG_ACQ_BASE+0x03)

#define NAP_ACQ_CODE_PHASE_WIDTH 12
#define NAP_ACQ_CARRIER_FREQ_WIDTH 20
#define NAP_ACQ_CODE_PHASE_UNITS_PER_CHIP (1<<(NAP_ACQ_CODE_PHASE_WIDTH-10))
#define NAP_ACQ_CARRIER_FREQ_UNITS_PER_HZ ((1<<NAP_ACQ_CARRIER_FREQ_WIDTH) / (float)SAMPLE_FREQ)

/*
 * Number of acquisition channel code phase taps that NAP
 * configuration was built with - read from configuration
 * flash at runtime in get_nap_parameters().
 */
extern u8 nap_acq_n_taps;

void nap_acq_load_wr_enable_blocking();
void nap_acq_load_wr_disable_blocking();
void nap_acq_init_pack(u8 pack[], u8 prn, u16 code_phase, s16 carrier_freq);
void nap_acq_init_wr_params_blocking(u8 prn, u16 code_phase, s16 carrier_freq);
void nap_acq_init_wr_disable_blocking();
void nap_acq_corr_unpack(u8 packed[], corr_t corrs[]);
void nap_acq_corr_rd_blocking(corr_t corrs[]);
void nap_acq_code_wr_blocking(u8 prn);

#endif /* SWIFTNAV_ACQ_CHANNEL_H */

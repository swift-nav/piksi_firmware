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

#ifndef SWIFT_CW_CHANNEL_H
#define SWIFT_CW_CHANNEL_H

#include <libopencm3/cm3/common.h>

#include "nap_common.h"
#include "../main.h"

#define NAP_REG_CW_BASE 0x02
#define NAP_REG_CW_INIT (NAP_REG_CW_BASE+0x00)
#define NAP_REG_CW_LOAD (NAP_REG_CW_BASE+0x01)
#define NAP_REG_CW_CORR (NAP_REG_CW_BASE+0x02)

#define CW_FREQ_WIDTH 20
#define CW_FREQ_UNITS_PER_HZ ((1<<CW_FREQ_WIDTH) / (float)SAMPLE_FREQ)

void cw_set_load_enable_blocking();
void cw_clear_load_enable_blocking();
void cw_pack_init(u8 pack[], s32 carrier_freq);
void cw_write_init_blocking(s32 carrier_freq);
void cw_disable_blocking();
void cw_unpack_corr(u8 packed[], corr_t* corrs);
void cw_read_corr_blocking(corr_t* corrs);

#endif /* SWIFT_CW_CHANNEL_H */

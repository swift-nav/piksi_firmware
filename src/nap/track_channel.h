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

#ifndef SWIFT_TRACK_CHANNEL_H
#define SWIFT_TRACK_CHANNEL_H

#include <libopencm3/cm3/common.h>

#include "nap_common.h"
#include "main.h"

/* SPI register ID's */
#define NAP_REG_TRACK_BASE  0x0A
#define TRACK_SIZE          5
#define TRACK_INIT_OFFSET   0x00
#define TRACK_UPDATE_OFFSET 0x01
#define TRACK_CORR_OFFSET   0x02
#define TRACK_PHASE_OFFSET  0x03
#define TRACK_CODE_OFFSET   0x04

/* Max number of track channels NAP configuration will be built with. */
#define MAX_TRACK_N_CHANNELS 12

/*
 * Actual number of track channels NAP configuration was built with - read from
 * configuration flash at runtime in nap_setup().
 */
extern u8 TRACK_N_CHANNELS;

/* NAP track channel parameters. */
#define TRACK_INIT_CODE_PHASE_WIDTH 14
#define TRACK_CARRIER_FREQ_WIDTH 24
#define TRACK_CODE_PHASE_RATE_WIDTH 29
#define TRACK_CODE_PHASE_FRACTIONAL_WIDTH 32
#define TRACK_INIT_CODE_PHASE_UNITS_PER_CHIP (1<<(TRACK_INIT_CODE_PHASE_WIDTH-10))
#define TRACK_CARRIER_FREQ_UNITS_PER_HZ ((1<<TRACK_CARRIER_FREQ_WIDTH) / (float)SAMPLE_FREQ)
#define TRACK_NOMINAL_CODE_PHASE_RATE (1<<(TRACK_CODE_PHASE_RATE_WIDTH-1))
#define TRACK_CODE_PHASE_RATE_UNITS_PER_HZ (TRACK_NOMINAL_CODE_PHASE_RATE / 1.023e6)
#define TRACK_CODE_PHASE_UNITS_PER_CHIP ((u64)1<<TRACK_CODE_PHASE_FRACTIONAL_WIDTH)

void track_pack_init(u8 pack[], u8 prn, s32 carrier_phase, u16 code_phase);
void track_write_init_blocking(u8 channel, u8 prn, s32 carrier_phase, u16 code_phase);
void track_pack_update(u8 pack[], s32 carrier_freq, u32 code_phase_rate);
void track_write_update_blocking(u8 channel, s32 carrier_freq, u32 code_phase_rate);
void track_unpack_corr(u8 packed[], u16* sample_count, corr_t corrs[]);
void track_read_corr_blocking(u8 channel, u16* sample_count, corr_t corrs[]);
void track_unpack_phase(u8 packed[], u32* carrier_phase, u64* code_phase);
void track_read_phase_blocking(u8 channel, u32* carrier_phase, u64* code_phase);
void track_write_code_blocking(u8 channel, u8 prn);

#endif /* SWIFT_TRACK_CHANNEL_H */

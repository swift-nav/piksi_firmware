/*
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
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

#ifndef SWIFTNAV_SWIFT_NAP_H
#define SWIFTNAV_SWIFT_NAP_H

#include <libopencm3/cm3/common.h>

#include "main.h"

#define SPI_ID_ACQ_BASE 0x01
#define SPI_ID_ACQ_INIT        (SPI_ID_ACQ_BASE+0x00)
#define SPI_ID_ACQ_LOAD_ENABLE (SPI_ID_ACQ_BASE+0x01)
#define SPI_ID_ACQ_CORR        (SPI_ID_ACQ_BASE+0x02)

#define SPI_ID_TRACK_BASE 0x04
#define TRACK_SIZE 4

#define TRACK_INIT_OFFSET   0x00
#define TRACK_UPDATE_OFFSET 0x01
#define TRACK_CORR_OFFSET   0x02
#define TRACK_PHASE_OFFSET  0x03

#define ACQ_N_TAPS 15
#define ACQ_CODE_PHASE_WIDTH 12
#define ACQ_CODE_PHASE_UNITS_PER_CHIP (1<<(ACQ_CODE_PHASE_WIDTH-10))
#define ACQ_CARRIER_FREQ_WIDTH 20
#define ACQ_CARRIER_FREQ_UNITS_PER_HZ ((1<<ACQ_CARRIER_FREQ_WIDTH) / (float)SAMPLE_FREQ)
/* NOTE: Minimum bin width 1/ACQ_CARRIER_FREQ_UNITS_PER_HZ (~16 Hz) */
#define ACQ_CARRIER_BIN_WIDTH 300

#define TRACK_CODE_PHASE_WIDTH 14
#define TRACK_CODE_PHASE_UNITS_PER_CHIP (1<<(TRACK_CODE_PHASE_WIDTH-10))
#define TRACK_CARRIER_FREQ_WIDTH 24
#define TRACK_CARRIER_FREQ_UNITS_PER_HZ ((1<<TRACK_CARRIER_FREQ_WIDTH) / (float)SAMPLE_FREQ)
#define TRACK_CODE_PHASE_RATE_WIDTH 29
#define TRACK_NOMINAL_CODE_PHASE_RATE (1<<(TRACK_CODE_PHASE_RATE_WIDTH-1))
#define TRACK_CODE_PHASE_RATE_UNITS_PER_HZ (TRACK_NOMINAL_CODE_PHASE_RATE / 1.023e6)

typedef struct {
  s32 I; s32 Q;
} corr_t;

void swift_nap_setup();
void swift_nap_reset();
void swift_nap_xfer(u8 spi_id, u8 n_bytes, u8 data_in[], u8 data_out[]);
void timing_strobe_setup();
u32 timing_count();
void timing_strobe(u32 falling_edge_count);
void acq_set_load_enable();
void acq_clear_load_enable();
void acq_write_init(u8 prn, u16 code_phase, s16 carrier_freq);
void acq_disable();
void acq_read_corr(corr_t corrs[]);

void do_one_acq(u8 prn, u16 code_phase, s16 carrier_freq, corr_t corrs[]);
void do_acq(u8 prn, float cp_min, float cp_max, float cf_min, float cf_max, float cf_bin_width, float* cp, float* cf, float* snr);

void track_write_init(u8 channel, u8 prn, s32 carrier_phase, u16 code_phase);
void track_write_update(u8 channel, s32 carrier_freq, u32 code_phase_rate);
void track_read_corr(u8 channel, corr_t corrs[]);

float propagate_code_phase(float code_phase, float carrier_freq, u32 n_samples);
void tracking_channel_init(u8 prn, u8 channel, float code_phase, float carrier_freq, u32 start_sample_count);

#endif

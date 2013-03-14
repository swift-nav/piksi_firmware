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

#ifndef SWIFTNAV_MANAGE_H
#define SWIFTNAV_MANAGE_H

#include <libopencm3/cm3/common.h>

#define ACQ_THRESHOLD 30.0
#define TRACK_THRESHOLD 2.0
#define TRACK_SNR_INIT_COUNT 5000
#define TRACK_SNR_THRES_COUNT 2000

#define ACQ_FULL_CF_MIN  -8500
#define ACQ_FULL_CF_MAX   8500
#define ACQ_FULL_CF_STEP  400
#define ACQ_FINE_CF_WIDTH 300
#define ACQ_FINE_CP_WIDTH 20
#define ACQ_FINE_CF_STEP  100

#define MANAGE_NO_CHANNELS_FREE 255

typedef enum {
  ACQ_MANAGE_START = 0,
  ACQ_MANAGE_DISABLED,
  ACQ_MANAGE_LOADING_COARSE,
  ACQ_MANAGE_RUNNING_COARSE,
  ACQ_MANAGE_LOADING_FINE,
  ACQ_MANAGE_RUNNING_FINE
} acq_manage_state_t;

typedef struct {
  acq_manage_state_t state;
  u8 prn;
  u32 coarse_timer_count;
  float coarse_snr, coarse_cf, coarse_cp;
  float fine_snr;
  u32 fine_timer_count;
} acq_manage_t;

#define ACQ_PRN_SKIP      0
#define ACQ_PRN_UNTRIED   1
#define ACQ_PRN_TRIED     2
#define ACQ_PRN_ACQUIRING 3
#define ACQ_PRN_TRACKING  4

typedef struct __attribute__((packed)) {
  s16 carrier_freq_min; /* Integer Hz. */
  s16 carrier_freq_max;
  u8 state;
} acq_prn_t;

void manage_acq_setup();
void manage_acq();
u8 manage_track_new_acq(float snr);
void manage_track();

#endif

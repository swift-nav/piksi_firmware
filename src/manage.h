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

#define ACQ_THRESHOLD 14.0
#define TRACK_THRESHOLD 2.0
#define TRACK_SNR_THRES_COUNT 1000

#define MANAGE_NO_CHANNELS_FREE 255

typedef enum {
  ACQ_MANAGE_START = 0,
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

typedef enum {
  ACQ_PRN_SKIP = 0,
  ACQ_PRN_UNTRIED,
  ACQ_PRN_TRIED,
  ACQ_PRN_ACQUIRING,
  ACQ_PRN_TRACKING
} acq_prn_state_t;

typedef struct {
  acq_prn_state_t state;
} acq_prn_t;

void manage_acq();
u8 manage_track_new_acq(float snr);
void manage_track();

#endif

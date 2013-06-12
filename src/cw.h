/*
 * Copyright (C) 2012 Colin Beighley <colinbeighley@gmail.com>
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

#ifndef SWIFTNAV_CW_H
#define SWIFTNAV_CW_H

#include <libopencm3/cm3/common.h>

/** \addtogroup cw
 * \{ */

#define SPECTRUM_LEN 301

typedef enum {
  CW_DISABLED = 0,
  CW_LOADING,
  CW_LOADING_DONE,
  CW_RUNNING,
  CW_RUNNING_DONE
} cw_status_t;

typedef struct {
  cw_status_t state;
  s32 freq_step, freq_min, freq_max;
  s32 freq;
	u16 count;
	u64 spectrum_power[SPECTRUM_LEN];
} cw_state_t;

typedef struct {
  float freq_min, freq_max, freq_step;
} cw_start_msg_t;

/** \} */

void cw_schedule_load(u32 count);
void cw_service_load_done();
u8 cw_get_load_done();
u8 cw_get_running_done();

void cw_setup();
void cw_start(float freq_min, float freq_max, float freq_bin_width);
void cw_service_irq();
void cw_send_result(float freq, u64 power);
void cw_get_spectrum_point(float* freq, u64* power, u16 index);

#endif

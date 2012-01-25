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

#ifndef SWIFTNAV_TRACK_H
#define SWIFTNAV_TRACK_H

#include <libopencm3/cm3/common.h>

float propagate_code_phase(float code_phase, float carrier_freq, u32 n_samples);
void tracking_channel_init(u8 prn, u8 channel, float code_phase, float carrier_freq, u32 start_sample_count);

#endif

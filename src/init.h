/*
 * Copyright (C) 2013 Fergus Noble <fergusnoble@gmail.com>
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

#ifndef SWIFTNAV_INIT_H
#define SWIFTNAV_INIT_H

#include <libopencm3/stm32/f4/rcc.h>

extern const clock_scale_t hse_16_368MHz_in_65_472MHz_out_3v3;
extern const clock_scale_t hse_16_368MHz_in_130_944MHz_out_3v3;
extern const clock_scale_t hse_16_368MHz_in_120_203MHz_out_3v3;

void init();

#endif


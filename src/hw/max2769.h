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

#ifndef SWIFTNAV_MAX2769_H
#define SWIFTNAV_MAX2769_H

#include <libopencm3/cm3/common.h>

#define MAX2769_CONF1   0x00
#define MAX2769_CONF2   0x01
#define MAX2769_CONF3   0x02
#define MAX2769_PLLCONF 0x03
#define MAX2769_DIV     0x04
#define MAX2769_FDIV    0x05
#define MAX2769_STRM    0x06
#define MAX2769_CLK     0x07
#define MAX2769_TEST1   0x08
#define MAX2769_TEST2   0x09

void max2769_write(u8 addr, u32 data);
void max2769_setup();

#endif

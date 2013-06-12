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

#include <libswiftnav/common.h>
#include <libswiftnav/pvt.h>

#include "track.h"

void nmea_gpgga(gnss_solution* soln, dops_t* dops);
void nmea_gpgsa(tracking_channel_t* chans, dops_t* dops);
void nmea_gpgsv(u8 n_used, navigation_measurement_t* nav_meas,
                gnss_solution* soln);


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

#ifndef SWIFTNAV_POSITION_H
#define SWIFTNAV_POSITION_H

#include <libswiftnav/common.h>
#include <libswiftnav/pvt.h>

/** \addtogroup position
 * \{ */

typedef enum {
  POSITION_UNKNOWN = 0,
  POSITION_GUESS,
  POSITION_STATIC,
  POSITION_FIX,
} position_quality_t;

/** \} */

extern position_quality_t position_quality;
extern gnss_solution position_solution;

void position_setup();
void position_updated();

#endif


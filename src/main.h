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

#ifndef SWIFTNAV_MAIN_H
#define SWIFTNAV_MAIN_H

/* GPS official value of Pi. */
#define PI 3.1415926535898
#define SAMPLE_FREQ 16368000
#define L1_HZ 1575.42e6
#define NOMINAL_CODE_PHASE_RATE_HZ 1.023e6

#define DO_EVERY(n, cmd) { \
  static u32 do_every_count = 0; \
  if (do_every_count % (n) == 0) { \
    cmd; \
  } \
  do_every_count++; \
}

#endif

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

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <libswiftnav/linear_algebra.h>

#include "position.h"
#include "timing.h"

#include "cfs/cfs.h"
#include "cfs/cfs-coffee.h"

position_quality_t position_quality = POSITION_UNKNOWN;
gnss_solution position_solution;

gps_time_t last_time;
double last_ecef[3];

void position_setup()
{
  int fd = cfs_open("posn", CFS_READ);
  if (fd != -1) {
    cfs_read(fd, &position_solution, sizeof(gnss_solution));
    if (position_solution.valid) {
      printf("Loaded last position solution from file: %.4f %.4f %.1f\n",
        position_solution.pos_llh[0]*(180/M_PI),
        position_solution.pos_llh[1]*(180/M_PI),
        position_solution.pos_llh[2]
      );
      position_quality = POSITION_GUESS;
      set_time(TIME_GUESS, position_solution.time);
      last_time = position_solution.time;
      memcpy(last_ecef, position_solution.pos_ecef, sizeof(last_ecef));
    } else {
      printf("Loaded position solution from file invalid\n");
    }
    cfs_close(fd);
  } else {
    printf("No position file present in flash, create an empty one\n");
    cfs_coffee_reserve("posn", sizeof(gnss_solution));
    cfs_coffee_configure_log("posn", 256, sizeof(gnss_solution));
  }
}

void position_updated()
{
  double temp[3];

  vector_subtract(3, position_solution.pos_ecef, last_ecef, temp);
  double dx = vector_norm(3, temp);

  double dt = gpsdifftime(position_solution.time, last_time);

  if (dt > 30*60 || dx > 10e3) {
    int fd = cfs_open("posn", CFS_WRITE);
    if (fd != -1) {
      if (cfs_write(fd, (void *)&position_solution, sizeof(position_solution)) != sizeof(position_solution))
        printf("Error writing to position file\n");
      else
        printf("Saved position to flash\n");
      cfs_close(fd);
    } else {
      printf("Error opening position file\n");
    }
    last_time = position_solution.time;
    memcpy(last_ecef, position_solution.pos_ecef, sizeof(last_ecef));
  }
}


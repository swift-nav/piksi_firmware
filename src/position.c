/*
 * Copyright (C) 2013-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include <string.h>

#include <libswiftnav/logging.h>
#include <libswiftnav/linear_algebra.h>

#include "position.h"
#include "timing.h"

#include "cfs/cfs-coffee.h"
#include "cfs/cfs.h"

position_quality_t position_quality = POSITION_UNKNOWN;
gnss_solution position_solution;

gps_time_t last_time;
double last_ecef[3];

/** \defgroup position Position
 * Maintains the position state of the receiver. Includes functions for saving
 * and loading a position estimate from filesystem.
 * \{ */

/** Get last saved position from file, or create position file if it does not
 * exist.
 */
void position_setup(void)
{
  int fd = cfs_open("posn", CFS_READ);

  if (fd != -1) {
    cfs_read(fd, &position_solution, sizeof(gnss_solution));
    if (position_solution.valid) {
      log_info("Loaded last position solution from file: %.4f %.4f %.1f",
               position_solution.pos_llh[0] * (180 / M_PI),
               position_solution.pos_llh[1] * (180 / M_PI),
               position_solution.pos_llh[2]);
      position_quality = POSITION_GUESS;
      set_time(TIME_GUESS, position_solution.time);
      last_time = position_solution.time;
      memcpy(last_ecef, position_solution.pos_ecef, sizeof(last_ecef));
    } else {
      log_warn("Loaded position solution from file invalid");
    }
    cfs_close(fd);
  } else {
    log_info("No position file present in flash, create an empty one");
    cfs_coffee_reserve("posn", sizeof(gnss_solution));
    cfs_coffee_configure_log("posn", 256, sizeof(gnss_solution));
  }
}

/** Save position to file. */
void position_updated(void)
{
  double temp[3];

  vector_subtract(3, position_solution.pos_ecef, last_ecef, temp);
  double dx = vector_norm(3, temp);

  double dt = gpsdifftime(&position_solution.time, &last_time);

  if (dt > 30 * 60 || dx > 10e3) {
    int fd = cfs_open("posn", CFS_WRITE);
    if (fd != -1) {
      if (cfs_write(fd, (void *)&position_solution,
                    sizeof(position_solution)) != sizeof(position_solution)) {
        log_error("Error writing to position file");
      } else {
        log_info("Saved position to flash");
      }
      cfs_close(fd);
    } else {
      log_error("Error opening position file");
    }
    last_time = position_solution.time;
    memcpy(last_ecef, position_solution.pos_ecef, sizeof(last_ecef));
  }

  position_quality = POSITION_FIX;
}

/** \} */

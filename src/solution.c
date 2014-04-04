/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libswiftnav/sbp_utils.h>

#include "nmea.h"
#include "sbp.h"
#include "solution.h"

void solution_send_sbp(gnss_solution *soln, dops_t *dops)
{
  /* Send GPS_TIME message first. */
  sbp_gps_time_t gps_time;
  sbp_make_gps_time(&gps_time, &soln->time, 0);
  sbp_send_msg(SBP_GPS_TIME, sizeof(gps_time), (u8 *) &gps_time);

  /* Position in LLH. */
  sbp_pos_llh_t pos_llh;
  sbp_make_pos_llh(&pos_llh, soln, 0);
  sbp_send_msg(SBP_POS_LLH, sizeof(pos_llh), (u8 *) &pos_llh);

  /* Velocity in NED. */
  sbp_vel_ned_t vel_ned;
  sbp_make_vel_ned(&vel_ned, soln, 0);
  sbp_send_msg(SBP_VEL_NED, sizeof(vel_ned), (u8 *) &vel_ned);

  DO_EVERY(10,
    sbp_dops_t sbp_dops;
    sbp_make_dops(&sbp_dops, dops);
    sbp_send_msg(SBP_DOPS, sizeof(sbp_dops_t), (u8 *) &sbp_dops);
  );
}

void solution_send_nmea(gnss_solution *soln, dops_t *dops,
                        u8 n, navigation_measurement_t *nm)
{
  nmea_gpgga(soln, dops);

  DO_EVERY(10,
    nmea_gpgsv(n, nm, soln);
  );
}


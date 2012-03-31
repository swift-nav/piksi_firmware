/*
 * Copyright (C) 2012 Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "pvt.h"
#include "track.h"
#include "ephemeris.h"
#include "tropo.h"
#include "coord_system.h"

void calc_loop_coeff(double BW, double zeta, double k, double *tau1, double *tau2)
{
  /* Solve for the natural frequency. */
  double omega_n = BW*8*zeta / (4*zeta*zeta + 1);

  *tau1 = k / (omega_n*omega_n);
  *tau2 = 2*zeta/omega_n;
}

void calc_navigation_measurement(u8 n_channels, channel_measurement_t meas[], navigation_measurement_t nav_meas[], double nav_time, ephemeris_t ephemerides[])
{
  double TOTs[n_channels];
  double mean_TOT = 0;

  for (u8 i=0; i<n_channels; i++) {
    TOTs[i] = 1e-3*meas[i].time_of_week_ms;
    TOTs[i] += meas[i].code_phase_chips / 1.023e6;
    TOTs[i] += (nav_time - meas[i].receiver_time) * meas[i].code_phase_rate / 1.023e6;

    nav_meas[i].TOT = TOTs[i];
    mean_TOT += TOTs[i];
    nav_meas[i].pseudorange_rate = NAV_C * -meas[i].carrier_freq / GPS_L1_HZ;
  }

  mean_TOT = mean_TOT/n_channels;

  double clock_err, clock_rate_err;

  double az, el;

  const double WPR_llh[3] = {D2R*37.038350, D2R*-122.141812, 376.7};
  double WPR_ecef[3];
  wgsllh2ecef(WPR_llh, WPR_ecef);

  for (u8 i=0; i<n_channels; i++) {
    nav_meas[i].pseudorange = (mean_TOT - TOTs[i])*NAV_C + NOMINAL_RANGE;

    calc_sat_pos(nav_meas[i].sat_pos, nav_meas[i].sat_vel, &clock_err, &clock_rate_err, &ephemerides[meas[i].prn], TOTs[i]);
    wgsecef2azel(nav_meas[i].sat_pos, WPR_ecef, &az, &el);

    nav_meas[i].pseudorange -= tropo_correction(el);
    nav_meas[i].pseudorange += clock_err*NAV_C;
    nav_meas[i].pseudorange_rate -= clock_rate_err*NAV_C;
  }
}


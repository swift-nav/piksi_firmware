/*
 * Copyright (C) 2010 Henry Hallam <henry@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "linear_algebra.h"
#include "ephemeris.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define NAV_OMEGAE_DOT 7.2921151467e-005
#define NAV_GM 3.986005e14

int calc_sat_pos(double pos[3], double vel[3],
             double *clock_err, double *clock_rate_err,
             const ephemeris_t *ephemeris,
             double time_of_transmit)
{
  /****************************************************************************
   * Calculate satellite position, velocity and clock offset from ephemeris
   * Taken from IS-GPS-200D, Section 20.3.3.3.3.1 and Table 20-IV
   *
   * Usage: unsigned int ProcessEphemeris(unsigned int week, double secs,
   *                              unsigned int sv, nav_info_t *Eph)
   *  week: GPS week number
   *  sec: GPS seconds of the week of time of transmission
   *  sv: Satellite vehicle number
   *    Eph: Ephemeris data structure located within channel structure
   ****************************************************************************/

  double tempd1 = 0, tempd2, tempd3;
  double tdiff;
  double a;     // semi major axis
  double ma, ma_dot;    // mean anomoly and first derivative (mean motion)
  double ea, ea_dot, ea_old;  // eccentric anomoly, first deriv, iteration var
  double einstein;    // relativistic correction
  double al, al_dot;    // argument of lattitude and first derivative
  double cal, cal_dot;    // corrected argument of lattitude and first deriv
  double r, r_dot;    // radius and first derivative
  double inc, inc_dot;    // inclination and first derivative
  double x, x_dot, y, y_dot;  // position in orbital plan and first derivatives
  double om, om_dot;    // omega and first derivatives


  // Satellite clock terms
  // Seconds from clock data reference time (toc)
  tdiff = time_of_transmit - ephemeris->toc;

  // Correct time for beginning or end of week crossovers and limit to +/- 302400
  if (tdiff > 302400.0) tdiff -= 604800.0;
    else if (tdiff < -302400.0) tdiff += 604800.0;

  *clock_err = ephemeris->af0 + tdiff * (ephemeris->af1 + tdiff * ephemeris->af2) - ephemeris->tgd;
  *clock_rate_err = ephemeris->af1 + 2.0 * tdiff * ephemeris->af2;

  // Seconds from the time from ephemeris reference epoch (toe)
  tdiff = (time_of_transmit - *clock_err) - (double) ephemeris->toe;
  //DBG("tdiff %10.3f\n", tdiff);

  if (tdiff > 302400.0) tdiff -= 604800.0;
    else if (tdiff < -302400.0) tdiff += 604800.0;

  // If tdiff is too large our ephemeris isn't valid, maybe we want to wait until we get a
  // new one? At least let's warn the user.
  // TODO: this doesn't exclude ephemerides older than a week so could be made better.
  if (abs(tdiff) > 4*3600)
    printf(" WARNING: using ephemeris older (or newer!) than 4 hours.\n");

  // Calculate position per IS-GPS-200D p 97 Table 20-IV
  a = ephemeris->sqrta * ephemeris->sqrta;  // [m] Semi-major axis
  ma_dot = sqrt (NAV_GM / (a * a * a)) + ephemeris->dn; // [rad/sec] Corrected mean motion
  ma = ephemeris->m0 + ma_dot * tdiff;  // [rad] Corrected mean anomaly

  // Iteratively solve for the Eccentric Anomaly (from Keith Alter and David Johnston)
  ea = ma;      // Starting value for E
  ea_old = ea + 1;

  while (fabs (ea - ea_old) > 1.0E-14)
  {
    ea_old = ea;
    tempd1 = 1.0 - ephemeris->ecc * cos (ea_old);
    ea = ea + (ma - ea_old + ephemeris->ecc * sin (ea_old)) / tempd1;
  }
  ea_dot = ma_dot / tempd1;

  // Relativistic correction term
  einstein = -4.442807633E-10 * ephemeris->ecc * ephemeris->sqrta * sin (ea);

  // Begin calc for True Anomaly and Argument of Latitude
  tempd2 = sqrt (1.0 - ephemeris->ecc * ephemeris->ecc);
  al = atan2 (tempd2 * sin (ea), cos (ea) - ephemeris->ecc) + ephemeris->w; // [rad] Argument of Latitude = True Anomaly + Argument of Perigee
  al_dot = tempd2 * ea_dot / tempd1;

  // Calculate corrected argument of latitude based on position
  cal = al + ephemeris->cus * sin (2.0 * al) + ephemeris->cuc * cos (2.0 * al);
  cal_dot =
    al_dot * (1.0 +
        2.0 * (ephemeris->cus * cos (2.0 * al) -
         ephemeris->cuc * sin (2.0 * al)));

  // Calculate corrected radius based on argument of latitude
  r =
    a * tempd1 + ephemeris->crc * cos (2.0 * al) +
    ephemeris->crs * sin (2.0 * al);
  r_dot =
    a * ephemeris->ecc * sin (ea) * ea_dot +
    2.0 * al_dot * (ephemeris->crs * cos (2.0 * al) -
        ephemeris->crc * sin (2.0 * al));

  // Calculate inclination based on argument of latitude
  inc =
    ephemeris->inc + ephemeris->inc_dot * tdiff +
    ephemeris->cic * cos (2.0 * al) + ephemeris->cis * sin (2.0 * al);
  inc_dot =
    ephemeris->inc_dot + 2.0 * al_dot * (ephemeris->cis * cos (2.0 * al) -
           ephemeris->cic * sin (2.0 * al));

  // Calculate position and velocity in orbital plane
  x = r * cos (cal);
  y = r * sin (cal);
  x_dot = r_dot * cos (cal) - y * cal_dot;
  y_dot = r_dot * sin (cal) + x * cal_dot;

  // Corrected longitude of ascenting node
  om_dot = ephemeris->omegadot - NAV_OMEGAE_DOT;
  om = ephemeris->omega0 + tdiff * om_dot - NAV_OMEGAE_DOT * ephemeris->toe;

  // Compute the satellite's position in Earth-Centered Earth-Fixed coordiates
  pos[0] = x * cos (om) - y * cos (inc) * sin (om);
  pos[1] = x * sin (om) + y * cos (inc) * cos (om);
  pos[2] = y * sin (inc);

  tempd3 = y_dot * cos (inc) - y * sin (inc) * inc_dot;

  // Compute the satellite's velocity in Earth-Centered Earth-Fixed coordiates
  vel[0] = -om_dot * pos[1] + x_dot * cos (om) - tempd3 * sin (om);
  vel[1] = om_dot * pos[0] + x_dot * sin (om) + tempd3 * cos (om);
  vel[2] = y * cos (inc) * inc_dot + y_dot * sin (inc);

  *clock_err += einstein;

  return 0;
}

double predict_range(double rx_pos[3],
                     double time_of_transmit,
                     ephemeris_t *ephemeris)
{
  double sat_pos[3];
  double sat_vel[3];
  double temp[3];
  double clock_err, clock_rate_err;

  calc_sat_pos(sat_pos, sat_vel, &clock_err, &clock_rate_err, ephemeris, time_of_transmit);

  vector_subtract(sat_pos, rx_pos, temp); // temp = sat_pos - rx_pos
  return vector_norm(temp);
}



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

#ifndef SWIFTLIB_EPHEMERIS_H
#define SWIFTLIB_EPHEMERIS_H

#include <stdint.h>

typedef struct {
  uint16_t wn;
  double tgd;
  double crs, crc, cuc, cus, cic, cis;
  double dn, m0, ecc, sqrta, omega0, omegadot, w, inc, inc_dot;
  double af0, af1, af2;
  uint32_t toe, toc;
  unsigned int valid:1;
  unsigned int healthy:1;


} ephemeris_t;

int calc_sat_pos(double pos[3], double vel[3],
             double *clock_err, double *clock_rate_err,
             const ephemeris_t *ephemeris,
             double time_of_transmit);

double predict_range(double rx_pos[3],
                     double time_of_transmit,
                     ephemeris_t *ephemeris);

#endif /* SWIFTLIB_EPHEMERIS_H */

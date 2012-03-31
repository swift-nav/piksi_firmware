/*
 * Copyright (c) 2010 Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTLIB_COORD_SYSTEM_H
#define SWIFTLIB_COORD_SYSTEM_H

void wgsllh2ecef(const double *llh, double *ecef);
void wgsecef2llh(const double const ecef[3], double llh[3]);

void wgsecef2ned(const double ecef[3], const double ref_ecef[3],
                 double ned[3]);
void wgsecef2ned_d(const double ecef[3], const double ref_ecef[3],
                   double ned[3]);

void wgsecef2azel(const double ecef[3], const double ref_ecef[3],
                  double* azimuth, double* elevation);

#endif /* SWIFTLIB_COORD_SYSTEM_H */


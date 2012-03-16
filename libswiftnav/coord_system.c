/*
 * Copyright (c) 2008, Scott Gleason
 * Copyright (c) 2010, Fergus Noble
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the authors' names nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <math.h>

#include "linear_algebra.h"
#include "coord_system.h"

#define A 6378137.0         /* semi-major axis */
#define B 6356752.3142      /* semi-minor axis */
#define E 0.0818191908426   /* eccentricity */
#define TOL 1.0E-13
#define DPI2 1.570796326794897

/*
 * Converts from WGS84 latitude, longitude and height into WGS X, Y and Z
 * From: Simo H. Laurila, "Electronic Surveying and Navigation", John Wiley & Sons (1976).
 * llh in [radians, radians, meters]
 */
void wgsllh2xyz(const double *llh, double *xyz)
{
  double n, d, nph, tempd;

  d = E * sin(llh[0]);
  n = A / sqrt(1. - d*d);
  nph = n + llh[2];

  tempd = nph * cos(llh[0]);
  xyz[0] = tempd * cos(llh[1]);
  xyz[1] = tempd * sin(llh[1]);
  xyz[2] = ((1 - E*E)*n + llh[2]) * sin(llh[0]);
}

/*
 * Converts from WGS84 X,Y Z to lat, lon, hgt
 * From: Simo H. Laurila, "Electronic Surveying and Navigation", John Wiley & Sons (1976).
 */
void wgsxyz2llh(double *xyz, double *llh)
{
  double n, d, nph, tempd, latx, hgtx;

  /* see if we are close to the Z axis */
  tempd = sqrt(xyz[0]*xyz[0] + xyz[1]*xyz[1]);
  if (tempd <= TOL)
  {
    /* we are very close to the Z axis */
    llh[0] = DPI2;
    if(xyz[2] < 0.0)
      llh[0] = -llh[0];
      
    llh[1] = 0.0;
    /* height at poles */
    llh[1] = fabs(xyz[2]) - B;
    
    return;
  }

  /* for all other cases */
  llh[0] = atan2(xyz[2], tempd);
  llh[1] = atan2(xyz[1], xyz[0]);

  /* height needs to be iterated */
  llh[2] = tempd / cos(llh[0]);
  latx = llh[0] + 1.0;
  hgtx = llh[2] + 1.0;

  while(fabs(llh[0] - latx) >= TOL || fabs(llh[2] - hgtx) >= 0.01)
  {
    latx = llh[0];
    hgtx = llh[2];
    d = E * sin(latx);
    n = A / sqrt(1.0 - d*d);
    llh[2] = tempd/cos(latx) - n;
    nph = n + llh[2];
    d = 1.0 - E*E*(n/nph);
    llh[0] = atan2(xyz[2], tempd*d);
  }
}

/*
 * Converts WGS84 ECEF coordinated into a local north, east, down frame
 * rotated around the reference WGS84 ECEF position. 
 *
 * NOTE: This returns the down coordinate __referenced to the centre of the 
 * Earth__ (so it is useful for velocities which do not need to be translated).
 * i.e. rotate but do NOT translate.
 */
void wgsxyz2ned_r(const double ecef[3], const double ref_ecef[3], double NED[3])
{
  double M[3][3];
  double ref_el, ref_az;
  double tempd;

  /* convert reference point to spherical earth centered coords */
  tempd = sqrt(ref_ecef[0]*ref_ecef[0] + ref_ecef[1]*ref_ecef[1]);
  ref_el = atan2(ref_ecef[2], tempd);
  ref_az = atan2(ref_ecef[1], ref_ecef[0]);

  M[0][0] = -sin(ref_el) * cos(ref_az);
  M[0][1] = -sin(ref_el) * sin(ref_az);
  M[0][2] = cos(ref_el);
  M[1][0] = -sin(ref_az);
  M[1][1] = cos(ref_az);
  M[1][2] = 0.0;
  M[2][0] = cos(ref_el) * cos(ref_az);
  M[2][1] = cos(ref_el) * sin(ref_az);
  M[2][2] = sin(ref_el);

  NED[0] = M[0][0]*ecef[0] + M[0][1]*ecef[1] + M[0][2]*ecef[2];
  NED[1] = M[1][0]*ecef[0] + M[1][1]*ecef[1] + M[1][2]*ecef[2];
  NED[2] = -(M[2][0]*ecef[0] + M[2][1]*ecef[1] + M[2][2]*ecef[2]);
}

/*
 * Converts WGS84 ECEF coordinated into a local north, east, down frame
 * rotated around the reference WGS84 ECEF position referenced to the reference point
 * altitude, i.e. rotate AND translate.
 *
 * NOTE: This returns the down coordinate referenced to the ref point altitude 
 * which is what you want for position but NOT for velocities!
 */
void wgsxyz2ned_rt(const double ecef[3], const double ref_ecef[3], double NED[3])
{
  wgsxyz2ned_r(ecef, ref_ecef, NED);
  NED[2] = NED[2] + vector_norm(ref_ecef);
}

/*
 * Converts ECEF satellite coordinates into azimuth and elevation
 * around the reference WGS84 ECEF position
 */
void satxyz2azel(const double satecef[3], const double rxecef[3], double* azimuth, double* elevation)
{
  double NED[3];
  double tempv[3];

  vector_subtract(satecef, rxecef, tempv);

  wgsxyz2ned_r(tempv, rxecef, NED);
  *azimuth = atan2(NED[1], NED[0]);
  *elevation = asin(-NED[2]/vector_norm(NED));
}


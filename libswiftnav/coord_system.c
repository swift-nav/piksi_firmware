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

#include <math.h>

#include "linear_algebra.h"
#include "coord_system.h"

/** \addtogroup lib
 * \{ */
/** \defgroup coord_system Coordinate systems
 * Functions used for converting between various coordinate systems.
 * References:
 *   -# <a href="http://bit.ly/H3HY4t">NIMA Technical Report TR8350.2</a>,
 *      "Department of Defense World Geodetic System 1984, Its Definition and
 *      Relationships With Local Geodetic Systems", Third Edition
 *   -# <a href="http://bit.ly/GShyW1">WGS84 Implementation Manual</a>,
 *      Eurocontrol, Version 2.4.
 *   -# <a href="http://bit.ly/GP1XFJ">
 *      Datum Transformations of GPS Positions</a>, Application Note, u-blox ag.
 *   -# <a href="http://en.wikipedia.org/wiki/Geodetic_system">
 *      Geodetic system</a>. In Wikipedia, The Free Encyclopedia.
 *      Retrieved 00:47, March 26, 2012.
 * \{ */

/** \defgroup WGS84_params WGS84 Parameters
 * Parameters defining the WGS84 ellipsoid. The ellipsoid is defined in terms
 * of the semi-major axis and the inverse flattening. We also calculate some
 * derived parameters which are useful for the implementation of the coordinate
 * transform functions.
 * \{ */
/** Semi-major axis of the Earth, \f$ a \f$, in meters.
 * This is a defining parameter of the WGS84 ellipsoid. */
#define A 6378137.0
/** Inverse flattening of the Earth, \f$ 1/f \f$.
 * This is a defining parameter of the WGS84 ellipsoid. */
#define IF 298.257223563
/** The flattening of the Earth, \f$ f \f$. */
#define F (1/IF)
/** Semi-minor axis of the Earth in meters, \f$ b = a(1-f) \f$. */
#define B (A*(1-F))
/** Eccentricity of the Earth, \f$ e \f$ where \f$ e^2 = 2f - f^2 \f$ */
#define E (sqrt(2*F - F*F))
/* \} */

/** Converts from WGS84 geodetic coordinates (latitude, longitude and height)
 * into WGS84 Earth Centered, Earth Fixed Cartesian (ECEF) coordinates
 * (X, Y and Z).
 *
 * Conversion from geodetic coordinates latitude, longitude and height
 * \f$(\phi, \lambda, h)\f$ into Cartesian coordinates \f$(X, Y, Z)\f$ can be
 * achieved with the following formulae:
 *
 * \f[ X = (N(\phi) + h) \cos{\phi}\cos{\lambda} \f]
 * \f[ Y = (N(\phi) + h) \cos{\phi}\sin{\lambda} \f]
 * \f[ Z = \left[(1-e^2)N(\phi) + h\right] \sin{\phi} \f]
 *
 * Where the 'radius of curvature', \f$ N(\phi) \f$, is defined as:
 *
 * \f[ N(\phi) = \frac{a}{\sqrt{1-e^2\sin^2 \phi}} \f]
 *
 * and \f$ a \f$ is the WGS84 semi-major axis and \f$ e \f$ is the WGS84
 * eccentricity. See \ref WGS84_params.
 *
 * \param llh  Geodetic coordinates to be converted, passed as
 *             [lat, lon, height] in [radians, radians, meters].
 * \param ecef Converted Cartesian coordinates are written into this array
 *             as [X, Y, Z], all in meters.
 */
void wgsllh2ecef(const double const llh[3], double ecef[3]) {
  double d = E * sin(llh[0]);
  double N = A / sqrt(1. - d*d);

  ecef[0] = (N + llh[2]) * cos(llh[0]) * cos(llh[1]);
  ecef[1] = (N + llh[2]) * cos(llh[0]) * sin(llh[1]);
  ecef[2] = ((1 - E*E)*N + llh[2]) * sin(llh[0]);
}

/** Converts from WGS84 Earth Centered, Earth Fixed (ECEF) Cartesian
 * coordinates (X, Y and Z) into WGS84 geodetic coordinates (latitude,
 * longitude and height).
 *
 * Conversion from Cartesian to geodetic coordinates is a much harder problem
 * than conversion from geodetic to Cartesian. There is no satisfactory closed
 * form solution but many different iterative approaches exist.
 *
 * Here we implement a relatively new algorithm due to Fukushima (2006) that is
 * very computationally efficient, not requiring any transcendental function
 * calls during iteration and very few divisions. It also exhibits cubic
 * convergence rates compared to the quadratic rate of convergence seen with
 * the more common algortihms based on the Newton-Raphson method.
 *
 * References:
 *   -# "A comparison of methods used in rectangular to Geodetic Coordinates
 *      Transformations", Burtch R. R. (2006), American Congress for Surveying
 *      and Mapping Annual Conference. Orlando, Florida.
 *   -# "Transformation from Cartesian to Geodetic Coordinates Accelerated by
 *      Halley’s Method", T. Fukushima (2006), Journal of Geodesy.
 *
 * \param ecef Cartesian coordinates to be converted, passed as [X, Y, Z],
 *             all in meters.
 * \param llh  Converted geodetic coordinates are written into this array as
 *             [lat, lon, height] in [radians, radians, meters].
 */
void wgsecef2llh(const double const ecef[3], double llh[3]) {
  /* Distance from polar axis. */
  const double p = sqrt(ecef[0]*ecef[0] + ecef[1]*ecef[1]);

  /* Compute longitude first, this can be done exactly. */
  if (p != 0)
    llh[1] = atan2(ecef[1], ecef[0]);
  else
    llh[1] = 0;

  /* If we are close to the pole then convergence is very slow, treat this is a
   * special case. */
  if (p < A*1e-16) {
    llh[0] = copysign(M_PI_2, ecef[2]);
    llh[2] = fabs(ecef[2]) - B;
    return;
  }

  /* Caluclate some other constants as defined in the Fukushima paper. */
  const double P = p / A;
  const double e_c = sqrt(1. - E*E);
  const double Z = fabs(ecef[2]) * e_c / A;

  /* Initial values for S and C correspond to a zero height solution. */
  double S = Z;
  double C = e_c * P;

  /* Neither S nor C can be negative on the first iteration so
   * starting prev = -1 will not cause and early exit. */
  double prev_C = -1;
  double prev_S = -1;

  double A_n, B_n, D_n, F_n;

  /* Iterate a maximum of 10 times. This should be way more than enough for all
   * sane inputs */
  for (int i=0; i<10; i++)
  {
    /* Calculate some intermmediate variables used in the update step based on
     * the current state. */
    A_n = sqrt(S*S + C*C);
    D_n = Z*A_n*A_n*A_n + E*E*S*S*S;
    F_n = P*A_n*A_n*A_n - E*E*C*C*C;
    B_n = 1.5*E*S*C*C*(A_n*(P*S - Z*C) - E*S*C);

    /* Update step. */
    S = D_n*F_n - B_n*S;
    C = F_n*F_n - B_n*C;

    /* The original algorithm as presented in the paper by Fukushima has a
     * problem with numerical stability. S and C can grow very large or small
     * and over or underflow a double. In the paper this is acknowledged and
     * the proposed resolution is to non-dimensionalise the equations for S and
     * C. However, this does not completely solve the problem. The author caps
     * the solution to only a couple of iterations and in this period over or
     * underflow is unlikely but as we require a bit more precision and hence
     * more iterations so this is still a concern for us.
     *
     * As the only thing that is important is the ratio T = S/C, my solution is
     * to divide both S and C by either S or C. The scaling is chosen such that
     * one of S or C is scaled to unity whilst the other is scaled to a value
     * less than one. By dividing by the larger of S or C we ensure that we do
     * not divide by zero as only one of S or C should ever be zero.
     *
     * This incurs an extra division each iteration which the author was
     * explicityl trying to avoid and it may be that this solution is just
     * reverting back to the method of iterating on T directly, perhaps this
     * bears more thought?
     */

    if (S > C) {
      C = C / S;
      S = 1;
    } else {
      S = S / C;
      C = 1;
    }

    /* Check for convergence and exit early if we have converged. */
    if (fabs(S - prev_S) < 1e-16 && fabs(C - prev_C) < 1e-16) {
      break;
    } else {
      prev_S = S;
      prev_C = C;
    }
  }

  A_n = sqrt(S*S + C*C);
  llh[0] = copysign(1.0, ecef[2]) * atan(S / (e_c*C));
  llh[2] = (p*e_c*C + fabs(ecef[2])*S - A*e_c*A_n) / sqrt(e_c*e_c*C*C + S*S);
}

/** Converts a vector in WGS84 Earth Centered, Earth Fixed (ECEF) Cartesian
 * coordinates to the local North, East, Down (NED) frame of a reference point,
 * also given in WGS84 ECEF coordinates.
 *
 * Note, this function only *rotates* the ECEF vector into the NED frame of
 * the reference point, as would be appropriate for e.g. a velocity vector. To
 * determine the distance between the point and the reference point in the NED
 * frame of the reference point, see \ref wgsecef2ned_d.
 *
 * \see wgsecef2ned_d.
 *
 * \param ecef      Cartesian coordinates of the point, passed as [X, Y, Z],
 *                  all in meters.
 * \param ref_ecef  Cartesian coordinates of the reference point, passed as
 *                  [X, Y, Z], all in meters.
 * \param ned       The North, East, Down vector is written into this array as
 *                  [N, E, D], all in meters.
 */
void wgsecef2ned(const double ecef[3], const double ref_ecef[3],
                 double ned[3]) {
  double M[3][3];
  double ref_el, ref_az;
  double tempd;

  /* Convert reference point to spherical earth centered coordinates. */
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

  ned[0] = M[0][0]*ecef[0] + M[0][1]*ecef[1] + M[0][2]*ecef[2];
  ned[1] = M[1][0]*ecef[0] + M[1][1]*ecef[1] + M[1][2]*ecef[2];
  ned[2] = -(M[2][0]*ecef[0] + M[2][1]*ecef[1] + M[2][2]*ecef[2]);
}

/** Returns the vector *to* a point given in WGS84 Earth Centered, Earth Fixed
 * (ECEF) Cartesian coordinates *from* a reference point, also given in WGS84
 * ECEF coordinates, in the local North, East, Down (NED) frame of the
 * reference point.
 *
 * \see wgsecef2ned.
 *
 * \param ecef      Cartesian coordinates of the point, passed as [X, Y, Z],
 *                  all in meters.
 * \param ref_ecef  Cartesian coordinates of the reference point, passed as
 *                  [X, Y, Z], all in meters.
 * \param ned       The North, East, Down vector is written into this array as
 *                  [N, E, D], all in meters.
 */
void wgsecef2ned_d(const double ecef[3], const double ref_ecef[3],
                   double ned[3]) {
  double tempv[3];
  vector_subtract(3, ecef, ref_ecef, tempv);
  wgsecef2ned(tempv, ref_ecef, ned);
}

/** Determine the azimuth and elevation of a point in WGS84 Earth Centered,
 * Earth Fixed (ECEF) Cartesian coordinates from a reference point given in
 * WGS84 ECEF coordinates.
 *
 * First the vector between the points is converted into the local North, East,
 * Down frame of the reference point. Then we can directly calculate the
 * azimuth and elevation.
 *
 * \param ecef      Cartesian coordinates of the point, passed as [X, Y, Z],
 *                  all in meters.
 * \param ref_ecef  Cartesian coordinates of the reference point from which the
 *                  azimuth and elevation is to be determined, passed as
 *                  [X, Y, Z], all in meters.
 * \param azimuth   Pointer to where to store the calculated azimuth output.
 * \param elevation Pointer to where to store the calculated elevation output.
 */
void wgsecef2azel(const double ecef[3], const double ref_ecef[3],
                  double* azimuth, double* elevation) {
  double ned[3];

  /* Calculate the vector from the reference point in the local North, East,
   * Down frame of the reference point. */
  wgsecef2ned_d(ecef, ref_ecef, ned);

  *azimuth = atan2(ned[1], ned[0]);
  /* atan2 returns angle in range [-pi, pi], usually azimuth is defined in the
   * range [0, 2pi]. */
  if (*azimuth < 0)
    *azimuth += 2*M_PI;

  *elevation = asin(-ned[2]/vector_norm(3, ned));
}

/** \} */
/** \} */


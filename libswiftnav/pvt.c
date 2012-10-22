/*
 * Copyright (C) 2010 Henry Hallam <henry@swift-nav.com>
 * Copyright (C) 2010 Matt Peddie <peddie@alum.mit.edu>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include <stdio.h>

#include "linear_algebra.h"
#include "coord_system.h"

#include "pvt.h"
#include "track.h"

static double vel_solve(double rx_vel[],
                        const navigation_measurement_t const nav_meas[GPS_NUM_SATS],
                        const u8 n_used,
                        const double const G[n_used][4],
                        const double const X[4][n_used])
{
  /* Velocity Solution
   *
   * G and X matrices already exist from the position
   * solution loop through valid measurements.  Here we form satellite
   * velocity and pseudorange rate vectors -- it's the same
   * prediction-error least-squares thing, but we do only one step.
  */

  double tempvX[n_used];
  double pdot_pred;

  for (u8 j = 0; j < n_used; j++) {
    /* Calculate predicted pseudorange rates from the satellite velocity
     * and the geometry matix G which contains normalised line-of-sight
     * vectors to the satellites.
     */
    pdot_pred = -vector_dot(3, G[j], nav_meas[j].sat_vel);

    /* The residual is due to the user's motion. */
    tempvX[j] = nav_meas[j].pseudorange_rate - pdot_pred;
  }

  /* Use X to map our pseudorange rate residuals onto the Jacobian update.
   *
   *   rx_vel[j] = X[j] . tempvX[j]
   */
  matrix_multiply(4, n_used, 1, (double *) X, (double *) tempvX, (double *) rx_vel);

  /* Return just the receiver clock bias. */
  return rx_vel[3];
}

void compute_dops(const double const H[4][4],
                  const double const pos_ecef[3],
                  dops_t *dops)
{
  double H_pos_diag[3];
  double H_ned[3];

  dops->gdop = dops->pdop = dops->tdop = dops->hdop = dops->vdop = 0;

  /* PDOP is the norm of the position elements of tr(H) */
  for (u8 i=0; i<3; i++) {
    dops->pdop += H[i][i];
    /* Also get the trace of H position states for use in HDOP/VDOP
     * calculations.
     */
    H_pos_diag[i] = H[i][i];
  }
  dops->pdop = sqrt(dops->pdop);

  /* TDOP is like PDOP but for the time state. */
  dops->tdop = sqrt(H[3][3]);

  /* Calculate the GDOP -- ||tr(H)|| = sqrt(PDOP^2 + TDOP^2) */
  dops->gdop = sqrt(dops->tdop*dops->tdop + dops->pdop*dops->pdop);

  /* HDOP and VDOP are Horizontal and Vertical; we need to rotate the
   * PDOP into NED frame and then take the separate components.
   */
  wgsecef2ned(H_pos_diag, pos_ecef, H_ned);
  dops->vdop = sqrt(H_ned[2]*H_ned[2]);
  dops->hdop = sqrt(H_ned[0]*H_ned[0] + H_ned[1]*H_ned[1]);
}


/* This function is the key to GPS solution, so it's commented
 * liberally.  It does a single step of a multi-dimensional
 * Newton-Raphson solution for the variables X, Y, Z (in ECEF) plus
 * the clock offset for each receiver used to make pseudorange
 * measurements.  The steps involved are roughly the following:
 *
 *     1. account for the Earth's rotation during transmission
 *
 *     2. Estimate the ECEF position for each satellite measured using
 *     the downloaded ephemeris
 *
 *     3. Compute the Jacobian of pseudorange versus estimated state.
 *     There's no explicit differentiation; it's done symbolically
 *     first and just coded as a "line of sight" vector.
 *
 *     4. Get the inverse of the Jacobian times its transpose.  This
 *     matrix is normalized to one, but it tells us the direction we
 *     must move the state estimate during this step.
 *
 *     5. Multiply this inverse matrix (H) by the transpose of the
 *     Jacobian (to yield X).  This maps the direction of our state
 *     error into a direction of pseudorange error.
 *
 *     6. Multiply this matrix (X) by the error between the estimated
 *     (ephemeris) position and the measured pseudoranges.  This
 *     yields a vector of corrections to our state estimate.  We apply
 *     these to our current estimate and recurse to the next step.
 *
 *     7. If our corrections are very small, we've arrived at a good
 *     enough solution.  Solve for the receiver's velocity (with
 *     vel_solve) and do some bookkeeping to pass the solution back
 *     out.
 */
static double pvt_solve(double rx_state[],
                        const u8 n_used,
                        const navigation_measurement_t const nav_meas[n_used],
                        double H[4][4])
{
  double p_pred[n_used];

  /* Vector of prediction errors */
  double omp[n_used];

  /* G is a geometry matrix tells us how our pseudoranges relate to
   * our state estimates -- it's the Jacobian of d(p_i)/d(x_j) where
   * x_j are x, y, z, Δt. */
  double G[n_used][4];
  double Gtrans[4][n_used];
  double GtG[4][4];

  /* H is the square of the Jacobian matrix; it tells us the shape of
     our error (or, if you prefer, the direction in which we need to
     move to get a better solution) in terms of the receiver state. */

  /* X is just H * Gtrans -- it maps our pseudoranges onto our
   * Jacobian update */
  double X[4][n_used];

  double tempv[3];
  double los[3];
  double xk_new[3];
  double tempd;
  double correction[4];

  for (u8 j=0; j<4; j++) {
    correction[j] = 0.0;
  }

  for (u8 j = 0; j < n_used; j++) {
    /* The satellite positions need to be corrected for earth's
     * rotation during the transmission time.  We base this correction
     * on the range between our receiver and satellite k */
    vector_subtract(3, rx_state, nav_meas[j].sat_pos, tempv);

    /* Magnitude of range vector converted into an approximate time in secs. */
    double tau = vector_norm(3, tempv) / NAV_C;
    /* Rotation of Earth during transit in radians. */
    double wEtau = NAV_OMEGAE_DOT * tau;

    /* Form rotation matrix about Z-axis for Earth's motion which will
     * adjust for the satellite's position at time (t-tau).
     */
    double rotm[3][3];
    rotm[0][0] = cos(wEtau);
    rotm[0][1] = sin(wEtau);
    rotm[0][2] = 0.0;
    rotm[1][0] = -sin(wEtau);
    rotm[1][1] = cos(wEtau);
    rotm[1][2] = 0.0;
    rotm[2][0] = 0.0;
    rotm[2][1] = 0.0;
    rotm[2][2] = 1.0;

    /* Result in xk_new, position of satellite k in ECEF. */
    matrix_multiply(3, 3, 1, (double *) rotm,
                             (double *) nav_meas[j].sat_pos,
                             (double *) xk_new);

    /* Predicted range from satellite position and estimated Rx position. */
    vector_subtract(3, rx_state, xk_new, tempv);
    p_pred[j] = vector_norm(3, tempv);

    /* omp means "observed minus predicted" range -- this is E, the
     * prediction error vector (or innovation vector in Kalman/LS
     * filtering terms).
     */
    omp[j] = nav_meas[j].pseudorange - p_pred[j];

    /* Line of sight vector. */
    vector_subtract(3, nav_meas[j].sat_pos, rx_state, los);

    /* Construct a geometry matrix.  Each row (satellite) is
     * independently normalized into a unit vector.
     */
    /* TODO: these aren't normalised now are they! But still
     * seems to work ok.
     */
    for (u8 i=0; i<3; i++) {
      los[i] = los[i] / p_pred[j];
      G[j][i] = -los[i];
    }

    /* Set time covariance to 1. */
    G[j][3] = 1;

  } /* End of channel loop. */

  /* Solve for position corrections using batch least-squares.  When
   * all-at-once least-squares estimation for a nonlinear problem is
   * mixed with numerical iteration (not time-series recursion, but
   * iteration on a single set of measurements), it's basically
   * Newton's method.  There's a reasonably clear explanation of this
   * on Wikipedia's article about GPS.
   */

  /* Gt := G^{T} */
  matrix_transpose(n_used, 4, (double *) G, (double *) Gtrans);
  /* GtG := G^{T} G */
  matrix_multiply(4, n_used, 4, (double *) Gtrans, (double *) G, (double *) GtG);
  /* H \elem \mathbb{R}^{4 \times 4} := GtG^{-1} */
  matrix_inverse(4, (const double *) GtG, (double *) H);
  /* X := H * G^{T} */
  matrix_multiply(4, 4, n_used, (double *) H, (double *) Gtrans, (double *) X);
  /* correction := X * E (= X * omp) */
  matrix_multiply(4, n_used, 1, (double *) X, (double *) omp, (double *) correction);

  /* Increment ecef estimate by the new corrections */
  for (u8 i=0; i<3; i++) {
    rx_state[i] += correction[i];
  }

  /* Set the Δt estimates according to this solution */
  for (u8 i=3; i<4; i++) {
    rx_state[i] = correction[i];
  }

  /* Look at the magnintude of the correction to see if
   * the solution has converged yet.
   */
  tempd = vector_norm(3, correction);
  if(tempd > 0.001) {
    /* The solution has not converged, return a negative value to
     * indicate that we should continue iterating.
     */
    return -tempd;
  }

  /* The solution has converged! */

  /* Perform the velocity solution. */
  vel_solve(&rx_state[4], nav_meas, n_used, (const double (*)[4]) G, (const double (*)[n_used]) X);

  return tempd;
}

u8 calc_PVT(const u8 n_used,
            const navigation_measurement_t const nav_meas[n_used],
            gnss_solution *soln,
            dops_t *dops)
{
  /* Initial state is the center of the Earth with zero velocity and zero
   * clock error, if we have some a priori position estimate we could use
   * that here to speed convergence a little on the first iteration.
   *
   *  rx_state format:
   *    pos[3], clock error, vel[3], intermediate freq error
   */
  static double rx_state[8];

  double H[4][4];

  soln->n_used = n_used; // Keep track of number of working channels

  /* reset state to zero !? */
  for(u8 i=4; i<8; i++) {
    rx_state[i] = 0;
  }

  double update;
  u8 iters;
  /* Newton-Raphson iteration. */
  for (iters=0; iters<PVT_MAX_ITERATIONS; iters++) {
    if ((update = pvt_solve(rx_state, n_used, nav_meas, H)) > 0) {
      break;
    }
  }

  /* Compute various dilution of precision metrics. */
  compute_dops((const double(*)[4])H, rx_state, dops);
  soln->err_cov[6] = dops->gdop;

  /* Populate error covariances according to layout in definition
   * of gnss_solution struct.
   */
  soln->err_cov[0] = H[0][0];
  soln->err_cov[1] = H[0][1];
  soln->err_cov[2] = H[0][2];
  soln->err_cov[3] = H[1][1];
  soln->err_cov[4] = H[1][2];
  soln->err_cov[5] = H[2][2];

  if (iters >= PVT_MAX_ITERATIONS) {
    printf("Position solution not available after %d iterations, giving up.\n", iters);
    /* Reset state if solution fails */
    rx_state[0] = 0;
    rx_state[1] = 0;
    rx_state[2] = 0;
    return -1;
  }

  /* Save as x, y, z. */
  for (u8 i=0; i<3; i++) {
    soln->pos_ecef[i] = rx_state[i];
    soln->vel_ecef[i] = rx_state[4+i];
  }

  /* Convert to lat, lon, hgt. */
  wgsecef2llh(rx_state, soln->pos_llh);

  /* Implicitly use the first receiver to calculate offset from GPS
   * TOW.  Maybe there's a better way to do this?  */
  /* TODO: what is this about? */
  soln->time -= rx_state[3] / NAV_C;

  return 0;
}


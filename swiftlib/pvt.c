/* 
 *  pvt.c
 *  Copyright 2010 Joby Energy, Inc.
 *  Henry Hallam, Matt Peddie
 */

#include <math.h>
#include <stdio.h>

#include "linear_algebra.h"
#include "coord_system.h"
#include "ephemeris.h"

#include "pvt.h"

void init_sat(gnss_satellite_state *sat, unsigned int prn, int recv_idx)
{
  sat->prn = prn;
  sat->az = sat->el = 0;
  sat->recv_idx = recv_idx;
  // DBG("Initializing PRN %02u\n",prn+1);
}


static double vel_solve(double rx_vel[],
                        const gnss_satellite_state sats[GPS_NUM_SATS],
                        unsigned int n_used,
                        unsigned int n_recv,
                        const double G[GPS_NUM_SATS][n_recv+3],
                        const double X[n_recv+3][GPS_NUM_SATS])
{
  /* Velocity Solution 
   *
   * G, Gtrans, X and H matrices already exist from the position
   * solution loop through valid measurements.  Here we form satellite
   * velocity and pseudorange rate vectors -- it's the same
   * prediction-error least-squares thing, but we do only one step.  
  */

  unsigned int n = 3+n_recv;

  double tempvX[GPS_NUM_SATS];
  double Vs[GPS_NUM_SATS][n];
  double pdot[GPS_NUM_SATS];
  double pdot_pred;
  unsigned int j, k;
  for (j = 0; j < n_used; j++) {
    pdot[j] = sats[j].pseudorange_rate; 

    /* Predict satellite velocity and subtract known antenna movement
     * due to rotation rate */
    for (k=0; k<3; k++) {
      Vs[j][k] = sats[j].vel[k];
    }
    Vs[j][3] = 0;

    pdot_pred = -(G[j][0] * Vs[j][0] + G[j][1] * Vs[j][1] + G[j][2] * Vs[j][2]);
    tempvX[j] = pdot[j] - pdot_pred; /* The residual is due to the user's motion */
  }
  matrix_multiply(n, n_used, 1, (double *) X, (double *) tempvX, (double *) rx_vel);

  /* Return just the bias of the first receiver */
  return rx_vel[3];
}

static void compute_dops(unsigned int n_recv,
                         double H[3+n_recv][3+n_recv],
                         double pos_xyz[3],
                         solution_plus *plus)
{
  unsigned int n = n_recv+3, i;
  double tr_H_pos[3], H_ned[3];
  plus->gdop = plus->pdop = plus->tdop = plus->hdop = plus->vdop = 0;

  /* PDOP is the norm of the position elements of tr(H) */
  plus->gdop = sqrt(plus->gdop);
  for (i=0; i<3; i++) {
    plus->pdop += H[i][i];
    /* Also get the trace of H position states for use in HDOP/VDOP
     * calculations */
    tr_H_pos[i] = H[i][i];
  }
  plus->pdop = sqrt(plus->pdop);
  /* TDOP is like PDOP but for time states. */
  for (i=3; i<n; i++) {
    plus->tdop += H[i][i];
  }
  plus->tdop = sqrt(plus->tdop);
  /* calculate the GDOP -- ||tr(H)|| = sqrt(PDOP^2 + TDOP^2) */
  plus->gdop = sqrt(plus->tdop + plus->pdop);
  /* HDOP and VDOP are Horizontal and Vertical; we need to rotate the
   * PDOP into NED frame and then take the separate components.  */
  wgsxyz2ned_r(tr_H_pos, pos_xyz, H_ned);
  plus->vdop = sqrt(H_ned[2]*H_ned[2]);
  plus->hdop = sqrt(H_ned[0]*H_ned[0] + H_ned[1]*H_ned[1]);
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

static double
pvt_solve(double rx_state[],
          const gnss_satellite_state sats[GPS_NUM_SATS],
          unsigned int n_used,
          unsigned int n_recv,
          const double W[GPS_NUM_SATS],
          double err_cov[7],
          solution_plus *plus)
{
  unsigned int n = 3+n_recv;

  unsigned int j;
  unsigned int i;

  double p_pred[GPS_NUM_SATS];
  /* Vector of prediction errors */
  double omp[GPS_NUM_SATS];
  double ompW[GPS_NUM_SATS];
  /* G is a geometry matrix tells us how our pseudoranges relate to
   * our state estimates -- it's the Jacobian of d(p_i)/d(x_j) where
   * x_j are x, y, z, Δt. */
  double G[GPS_NUM_SATS][n];
  double Gtrans[n][GPS_NUM_SATS];
  double GtG[n][n];
  /* H is the square of the Jacobian matrix; it tells us the shape of
     our error (or, if you prefer, the direction in which we need to
     move to get a better solution) in terms of the receiver state. */
  double H[n][n];
  /* X is just H * Gtrans -- it maps our pseudoranges onto our
   * Jacobian update */
  double X[n][GPS_NUM_SATS];
  double tempv[3];
  double rotm[3][3];
  double los[3];
  double xk_new[3];
  double tempd, x0xk_mag, tau, wEtau;
  double correction[n];

  for (j=0; j<n; j++) {
    correction[j] = 0.0;
  }

  for (j = 0; j < n_used; j++) {
    /* The satellite positions need to be corrected for earth's
     * rotation during the transmission time.  We base this correction
     * on the range between our receiver and satellite k */
    vector_subtract(rx_state, sats[j].pos, tempv); 
    x0xk_mag = vector_norm (tempv); // [m] magnitude of range vector
    tau = x0xk_mag / NAV_C; // [sec] Convert into an approximate time
    wEtau = NAV_OMEGAE_DOT * tau; // [rad] rotation of Earth during transit

    /* Form rotation matrix about Z-axis for Earth's motion which will
     * adjust for the satellite's position at time (t-tau) */
    rotm[0][0] = cos(wEtau);
    rotm[0][1] = sin(wEtau);
    rotm[0][2] = 0.0;
    rotm[1][0] = -sin(wEtau);
    rotm[1][1] = cos(wEtau);
    rotm[1][2] = 0.0;
    rotm[2][0] = 0.0;
    rotm[2][1] = 0.0;
    rotm[2][2] = 1.0;
    // result in xk_new, position of satellite k in ECEF
    matrix_multiply(3, 3, 1, (double *) rotm, 
                    (double *) sats[j].pos, 
                    (double *) xk_new);

    // predicted range from satellite position and estimated Rx position
    vector_subtract(rx_state, xk_new, tempv);
    p_pred[j] = vector_norm(tempv);

    /* omp means "observed minus predicted" range -- this is E, the
     * prediction error vector (or innovation vector in Kalman/LS
     * filtering terms). */
    omp[j] = sats[j].pseudorange - p_pred[j];

    // line of sight vector
    vector_subtract (sats[j].pos, rx_state, los);

    /* Construct a geometry matrix.  Each row (satellite) is
     * independently normalized into a unit vector.  */
    for (i=0; i<3; i++) {
      los[i] = los[i] / p_pred[j];
      G[j][i] = -1 * los[i];
    }

    /* Set time covariance to 0 for other receivers and 1 for this
     * receiver */
    for (i=0; i<n_recv; i++) {
      G[j][3+i] = 0;
    }
    G[j][3+sats[j].recv_idx] = 1;

  } // end of channel loop

  /* Solve for position corrections using batch least-squares.  When
   * all-at-once least-squares estimation for a nonlinear problem is
   * mixed with numerical iteration (not time-series recursion, but
   * iteration on a single set of measurements), it's basically
   * Newton's method.  There's a reasonably clear explanation of this
   * on Wikipedia's article about GPS.  */

  /* ompW := W omp, W diagonal */
  diag_matrix_vector_multiply(n_used, ompW, W, omp);
  /* Gt := G^{T} */
  matrix_transpose(n_used, n, (double *) G, (double *) Gtrans);
  /* GtG := G^{T} G */
  matrix_multiply(n, n_used, n, (double *) Gtrans, (double *) G, (double *) GtG);
  /* H \elem \mathbb{R}^{3+n_recv \times 3+n_recv} := GtG^{-1} */
  matrix_inverse(n, (const double *) GtG, (double *) H);
  /* X := H * G^{T} */
  matrix_multiply(n, n, n_used, (double *) H, (double *) Gtrans, (double *) X);
  /* correction := X * E (= X * omp) */
  matrix_multiply(n, n_used, 1, (double *) X, (double *) ompW, (double *) correction);

  /* Increment xyz estimate by the new corrections */
  for (i=0; i<3; i++) {
    rx_state[i] += correction[i];
  }
  
  /* Set the Δt estimates according to this solution */
  for (i=3; i<n; i++) {
    rx_state[i] = correction[i];
  }

  tempd = vector_norm(correction);
  if(tempd > 0.001) {
    return -tempd;
  }

  /* Compute various dilution of precision metrics. */
  compute_dops(n_recv, H, rx_state, plus);
  err_cov[6] = plus->gdop;

  /* Populate error covariances according to layout in definition
   * of gnss_solution struct (gnss_external.h) */
  err_cov[0] = H[0][0];
  err_cov[1] = H[0][1];
  err_cov[2] = H[0][2];
  err_cov[3] = H[1][1];
  err_cov[4] = H[1][2];
  err_cov[5] = H[2][2];

  plus->n_used = n_used;
  plus->n_recv = n_recv;
  for (i=0; i<n_used; i++) {
    plus->innovation[i] = omp[i];
  }

  vel_solve(&rx_state[n], sats, n_used, n_recv, (const double (*)[n]) G, (const double (*)[GPS_NUM_SATS]) X);
  return tempd;
}   
   
int
calc_PVT(gnss_solution *soln, 
         unsigned int n_used,
         unsigned int n_recv,    
         const gnss_satellite_state sats[GPS_NUM_SATS],
         const double W[GPS_NUM_SATS],
         double rx_time[n_recv],
         double rx_freq_bias[n_recv],
         solution_plus *plus)       
{
  unsigned int n = 3+n_recv;
  unsigned int state_len = n*2;

  /* Initial receiver state estimate is at Woodpecker with zero
   * velocity.  This is a minor improvement over using ECEF (0, 0, 0)
   * and shouldn't cause any problems until Joby Turkmenistan starts
   * using this code (and even then, probably not).  
   */

  // pos[3], clock err, vel[3], intermediate freq err
  static double rx_state[6+2*GNSS_MAX_RECEIVERS] = {-2712219, -4316338, 3820996}; 

  unsigned int max_iterations, i;
  double update;

  soln->num_PVT = n_used; // Keep track of number of working channels

  /* Newton-Raphson iteration. */
  max_iterations = 20;

  for(i=n; i<state_len; i++) {
    rx_state[i] = 0;
  }

  for (i = 0; i < max_iterations; i++) {
    if ((update = pvt_solve(rx_state, sats, n_used, n_recv, W, soln->err_cov, plus)) > 0) {
      break;
    }
  }

  if (i >= max_iterations) {
     printf("Position solution not available after %d iterations, giving up.\n", i);
    // DBG("||correction|| was %lf\n", -update);
    /* Reset state to woodpecker if solution fails */
    rx_state[0] = -2712219;
    rx_state[1] = -4316338;
    rx_state[2] = 3820996;
    return -1;
  }

  // save as x y z
  for (i = 0; i < 3; i++) {
    soln->pos_xyz[i] = rx_state[i];
    soln->vel_xyz[i] = rx_state[n+i];
  }

  // convert to lat/lon/hgt
  wgsxyz2llh(rx_state, soln->pos_llh);
  /* Implicitly use the first receiver to calculate offset from GPS
   * TOW.  Maybe there's a better way to do this?  */
  soln->time -= rx_state[3] / NAV_C;
  // Why yes, there is:
 
  /* The question was unclear.  What I meant was: which time do we
   * give to the autopilot as "the" solution time?  "soln->time"
   * hasn't changed.  */
  if (rx_time) {
    for (int r = 0; r < (int)n_recv; r++)
      for (i = 0; i < n_used; i++)
        if (sats[i].recv_idx == r) {
          rx_time[r]=sats[i].totc + sats[i].pseudorange/NAV_C - rx_state[3+r]/NAV_C;
          break;
        }
  }
  
  if (rx_freq_bias) {
    for (int r = 0; r < (int)n_recv; r++) {
      rx_freq_bias[r]=(rx_state[n+3+r]/NAV_C)*GPS_L1_HZ;
    }
  }

  return 0;
}


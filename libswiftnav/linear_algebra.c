/*
 * Copyright (C) 2012 Matt Peddie <peddie@alum.mit.edu>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include "common.h"

#include "linear_algebra.h"

/* Todo(MP) -- Implement fast linear solve (all-in-one) with Cholesky
 * decomposition: we want to solve $A^{T} A \hat{x} = A^{T} y */

/** \addtogroup lib
 * \{ */
/** \defgroup linear_algebra Linear Algebra
 * Basic linear algebra routines.
 * References:
 *   -# <a href="http://en.wikipedia.org/wiki/Invertible_matrix">
 *      Invertible Matrix</a>. In Wikipedia, The Free Encyclopedia.
 *      Retrieved 00:47, March 26, 2012.
 * \{ */

/** \defgroup numerical_params Numerical Parameters
 * Parameters that define singularity, convergence, and similar
 * numerical properties.
 * \{ */

/** Tolerance for matrix inverses.
 * If the determinant is smaller than this value, we consider it
 * singular and error out.
 */
#define MATRIX_EPSILON (1e-11)

/* \} */

/** \defgroup matrices Matrix Mathematics
 *  Routines for working with matrices.
 * \{ */

/** Invert a 2x2 matrix.
 *  Calculate the inverse of a 2x2 matrix: \f$ b := a^{-1} \f$
 *
 *  \param a    The matrix to invert (input)
 *  \param b    Where to put the inverse (output)
 *
 *  \return     -1 if a is singular; 0 otherwise.
 */
static inline int inv2(const double *a, double *b) {
  double det = a[0]*a[3] - a[1]*a[2];
  if (det < MATRIX_EPSILON)
    return -1;
  b[0] = a[3]/det;
  b[1] = -a[1]/det;
  b[2] = -a[2]/det;
  b[3] = a[0]/det;
  return 0;
}

/** Invert a 3x3 matrix.
 *  Calculate the inverse of a 3x3 matrix: \f$ b := a^{-1} \f$
 *
 *  \param a    The matrix to invert (input)
 *  \param b    Where to put the inverse (output)
 *
 *  \return     -1 if a is singular; 0 otherwise.
 */
static inline int inv3(const double *a, double *b) {
  double det = ((a[3*1 + 0]*-(a[3*0 + 1]*a[3*2 + 2]-a[3*0 + 2]*a[3*2 + 1])
                 +a[3*1 + 1]*(a[3*0 + 0]*a[3*2 + 2]-a[3*0 + 2]*a[3*2 + 0]))
                +a[3*1 + 2]*-(a[3*0 + 0]*a[3*2 + 1]-a[3*0 + 1]*a[3*2 + 0]));

  if (det < MATRIX_EPSILON)
    return -1;

  b[3*0 + 0] = (a[3*1 + 1]*a[3*2 + 2]-a[3*1 + 2]*a[3*2 + 1])/det;
  b[3*1 + 0] = -(a[3*1 + 0]*a[3*2 + 2]-a[3*1 + 2]*a[3*2 + 0])/det;
  b[3*2 + 0] = (a[3*1 + 0]*a[3*2 + 1]-a[3*1 + 1]*a[3*2 + 0])/det;

  b[3*0 + 1] = -(a[3*0 + 1]*a[3*2 + 2]-a[3*0 + 2]*a[3*2 + 1])/det;
  b[3*1 + 1] = (a[3*0 + 0]*a[3*2 + 2]-a[3*0 + 2]*a[3*2 + 0])/det;
  b[3*2 + 1] = -(a[3*0 + 0]*a[3*2 + 1]-a[3*0 + 1]*a[3*2 + 0])/det;

  b[3*0 + 2] = (a[3*0 + 1]*a[3*1 + 2]-a[3*0 + 2]*a[3*1 + 1])/det;
  b[3*1 + 2] = -(a[3*0 + 0]*a[3*1 + 2]-a[3*0 + 2]*a[3*1 + 0])/det;
  b[3*2 + 2] = (a[3*0 + 0]*a[3*1 + 1]-a[3*0 + 1]*a[3*1 + 0])/det;

  return 0;
}

/** Invert a 4x4 matrix.
 *  Calculate the inverse of a 4x4 matrix: \f$ b := a^{-1} \f$
 *
 *  \param a    The matrix to invert (input)
 *  \param b    Where to put the inverse (output)
 *
 *  \return     -1 if a is singular; 0 otherwise.
 */
static inline int inv4(const double *a, double *b) {
  double det = (((a[4*1 + 0]*-((a[4*2 + 1]*-(a[4*0 + 2]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 2])+a[4*2 + 2]*(a[4*0 + 1]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 1]))+a[4*2 + 3]*-(a[4*0 + 1]*a[4*3 + 2]-a[4*0 + 2]*a[4*3 + 1]))+a[4*1 + 1]*((a[4*2 + 0]*-(a[4*0 + 2]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 2])+a[4*2 + 2]*(a[4*0 + 0]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 0]))+a[4*2 + 3]*-(a[4*0 + 0]*a[4*3 + 2]-a[4*0 + 2]*a[4*3 + 0])))+a[4*1 + 2]*-((a[4*2 + 0]*-(a[4*0 + 1]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 1])+a[4*2 + 1]*(a[4*0 + 0]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 0]))+a[4*2 + 3]*-(a[4*0 + 0]*a[4*3 + 1]-a[4*0 + 1]*a[4*3 + 0])))+a[4*1 + 3]*((a[4*2 + 0]*-(a[4*0 + 1]*a[4*3 + 2]-a[4*0 + 2]*a[4*3 + 1])+a[4*2 + 1]*(a[4*0 + 0]*a[4*3 + 2]-a[4*0 + 2]*a[4*3 + 0]))+a[4*2 + 2]*-(a[4*0 + 0]*a[4*3 + 1]-a[4*0 + 1]*a[4*3 + 0])));

  if (det < MATRIX_EPSILON)
    return -1;

  b[4*0 + 0] = ((a[4*2 + 1]*-(a[4*1 + 2]*a[4*3 + 3]-a[4*1 + 3]*a[4*3 + 2])+a[4*2 + 2]*(a[4*1 + 1]*a[4*3 + 3]-a[4*1 + 3]*a[4*3 + 1]))+a[4*2 + 3]*-(a[4*1 + 1]*a[4*3 + 2]-a[4*1 + 2]*a[4*3 + 1]))/det;
  b[4*1 + 0] = -((a[4*2 + 0]*-(a[4*1 + 2]*a[4*3 + 3]-a[4*1 + 3]*a[4*3 + 2])+a[4*2 + 2]*(a[4*1 + 0]*a[4*3 + 3]-a[4*1 + 3]*a[4*3 + 0]))+a[4*2 + 3]*-(a[4*1 + 0]*a[4*3 + 2]-a[4*1 + 2]*a[4*3 + 0]))/det;
  b[4*2 + 0] = ((a[4*2 + 0]*-(a[4*1 + 1]*a[4*3 + 3]-a[4*1 + 3]*a[4*3 + 1])+a[4*2 + 1]*(a[4*1 + 0]*a[4*3 + 3]-a[4*1 + 3]*a[4*3 + 0]))+a[4*2 + 3]*-(a[4*1 + 0]*a[4*3 + 1]-a[4*1 + 1]*a[4*3 + 0]))/det;
  b[4*3 + 0] = -((a[4*2 + 0]*-(a[4*1 + 1]*a[4*3 + 2]-a[4*1 + 2]*a[4*3 + 1])+a[4*2 + 1]*(a[4*1 + 0]*a[4*3 + 2]-a[4*1 + 2]*a[4*3 + 0]))+a[4*2 + 2]*-(a[4*1 + 0]*a[4*3 + 1]-a[4*1 + 1]*a[4*3 + 0]))/det;

  b[4*0 + 1] = -((a[4*2 + 1]*-(a[4*0 + 2]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 2])+a[4*2 + 2]*(a[4*0 + 1]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 1]))+a[4*2 + 3]*-(a[4*0 + 1]*a[4*3 + 2]-a[4*0 + 2]*a[4*3 + 1]))/det;
  b[4*1 + 1] = ((a[4*2 + 0]*-(a[4*0 + 2]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 2])+a[4*2 + 2]*(a[4*0 + 0]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 0]))+a[4*2 + 3]*-(a[4*0 + 0]*a[4*3 + 2]-a[4*0 + 2]*a[4*3 + 0]))/det;
  b[4*2 + 1] = -((a[4*2 + 0]*-(a[4*0 + 1]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 1])+a[4*2 + 1]*(a[4*0 + 0]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 0]))+a[4*2 + 3]*-(a[4*0 + 0]*a[4*3 + 1]-a[4*0 + 1]*a[4*3 + 0]))/det;
  b[4*3 + 1] = ((a[4*2 + 0]*-(a[4*0 + 1]*a[4*3 + 2]-a[4*0 + 2]*a[4*3 + 1])+a[4*2 + 1]*(a[4*0 + 0]*a[4*3 + 2]-a[4*0 + 2]*a[4*3 + 0]))+a[4*2 + 2]*-(a[4*0 + 0]*a[4*3 + 1]-a[4*0 + 1]*a[4*3 + 0]))/det;

  b[4*0 + 2] = ((a[4*1 + 1]*-(a[4*0 + 2]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 2])+a[4*1 + 2]*(a[4*0 + 1]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 1]))+a[4*1 + 3]*-(a[4*0 + 1]*a[4*3 + 2]-a[4*0 + 2]*a[4*3 + 1]))/det;
  b[4*1 + 2] = -((a[4*1 + 0]*-(a[4*0 + 2]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 2])+a[4*1 + 2]*(a[4*0 + 0]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 0]))+a[4*1 + 3]*-(a[4*0 + 0]*a[4*3 + 2]-a[4*0 + 2]*a[4*3 + 0]))/det;
  b[4*2 + 2] = ((a[4*1 + 0]*-(a[4*0 + 1]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 1])+a[4*1 + 1]*(a[4*0 + 0]*a[4*3 + 3]-a[4*0 + 3]*a[4*3 + 0]))+a[4*1 + 3]*-(a[4*0 + 0]*a[4*3 + 1]-a[4*0 + 1]*a[4*3 + 0]))/det;
  b[4*3 + 2] = -((a[4*1 + 0]*-(a[4*0 + 1]*a[4*3 + 2]-a[4*0 + 2]*a[4*3 + 1])+a[4*1 + 1]*(a[4*0 + 0]*a[4*3 + 2]-a[4*0 + 2]*a[4*3 + 0]))+a[4*1 + 2]*-(a[4*0 + 0]*a[4*3 + 1]-a[4*0 + 1]*a[4*3 + 0]))/det;

  b[4*0 + 3] = -((a[4*1 + 1]*-(a[4*0 + 2]*a[4*2 + 3]-a[4*0 + 3]*a[4*2 + 2])+a[4*1 + 2]*(a[4*0 + 1]*a[4*2 + 3]-a[4*0 + 3]*a[4*2 + 1]))+a[4*1 + 3]*-(a[4*0 + 1]*a[4*2 + 2]-a[4*0 + 2]*a[4*2 + 1]))/det;
  b[4*1 + 3] = ((a[4*1 + 0]*-(a[4*0 + 2]*a[4*2 + 3]-a[4*0 + 3]*a[4*2 + 2])+a[4*1 + 2]*(a[4*0 + 0]*a[4*2 + 3]-a[4*0 + 3]*a[4*2 + 0]))+a[4*1 + 3]*-(a[4*0 + 0]*a[4*2 + 2]-a[4*0 + 2]*a[4*2 + 0]))/det;
  b[4*2 + 3] = -((a[4*1 + 0]*-(a[4*0 + 1]*a[4*2 + 3]-a[4*0 + 3]*a[4*2 + 1])+a[4*1 + 1]*(a[4*0 + 0]*a[4*2 + 3]-a[4*0 + 3]*a[4*2 + 0]))+a[4*1 + 3]*-(a[4*0 + 0]*a[4*2 + 1]-a[4*0 + 1]*a[4*2 + 0]))/det;
  b[4*3 + 3] = ((a[4*1 + 0]*-(a[4*0 + 1]*a[4*2 + 2]-a[4*0 + 2]*a[4*2 + 1])+a[4*1 + 1]*(a[4*0 + 0]*a[4*2 + 2]-a[4*0 + 2]*a[4*2 + 0]))+a[4*1 + 2]*-(a[4*0 + 0]*a[4*2 + 1]-a[4*0 + 1]*a[4*2 + 0]))/det;

  return 0;
}

/* Apparently there is no C code online that simply inverts a general
 * matrix without having to include GSL or something.  Therefore, I
 * give you Gaussian Elimination for matrix inversion.  --MP */

/* Helper function for rref */
static void row_swap(double *a, double *b, u32 size) {
  double tmp;
  for(u32 i = 0; i < size; i++) {
    tmp = a[i];
    a[i] = b[i];
    b[i] = tmp;
  }
}

/* rref is "reduced row echelon form" -- a helper function for the
 * gaussian elimination code. */
static int rref(u32 order, u32 cols, double *m) {
  int i, j, k, maxrow;
  double tmp;

  for (i = 0; i < (int) order; i++) {
    maxrow = i;
    for (j = i+1; j < (int) order; j++) {
      /* Find the maximum pivot */
      if (fabs(m[j*cols+i]) > fabs(m[maxrow*cols+i]))
        maxrow = j;
    }
    row_swap(&m[i*cols], &m[maxrow*cols], cols);
    if (fabs(m[i*cols+i]) <= MATRIX_EPSILON) {
      /* If we've eliminated our diagonal element, it means our matrix
       * isn't full-rank.  Pork chop sandwiches!  */
      return -1;
    }
    for (j = i+1; j < (int) order; j++) {
      /* Elimination of column i */
      tmp = m[j*cols+i] / m[i*cols+i];
      for (k = i; k < (int) cols; k++) {
        m[j*cols+k] -= m[i*cols+k] * tmp;
      }
    }
  }
  for (i = order-1; i >= 0; i--) {
    /* Back-substitution */
    tmp = m[i*cols+i];
    for (j = 0; j < i; j++) {
      for (k = cols-1; k > i-1; k--) {
        m[j*cols+k] -= m[i*cols+k] * m[j*cols+i] / tmp;
      }
    }
    m[i*cols+i] /= tmp;
    for (j = order; j < (int) cols; j++) {
      /* Normalize row */
      m[i*cols+j] /= tmp;
    }
  }
  return 0;
}

/** Invert a square matrix.
 *  Calculate the inverse of a square matrix: \f$ B := A^{-1} \f$,
 *  where \f$A\f$ and \f$B\f$ are matrices on \f$\mathbb{R}^{n \times
 *  n}\f$. For matrices size 4x4 and smaller, this is done by
 *  autogenerated hard-coded routines.  For larger matrices, this is
 *  done by Gauss-Jordan elimination (which is \f$ O(n^{3}) \f$).
 *
 *  \param n            The rank of a and b
 *  \param a            The matrix to invert (input)
 *  \param b            Where to put the inverse (output)
 *
 *  \return     -1 if a is singular; 0 otherwise.
 */
inline int matrix_inverse(u32 n, const double const *a, double *b) {
  /* This function is currently only used to do a linear least-squares
   * solve for x, y, z and t in the navigation filter.  Gauss-Jordan
   * elimination is not the most efficient way to do this.  In the
   * ideal case, we'd use the Cholesky decomposition to compute the
   * least-squares fit.  (This may apply also to a least-norm fit if
   * we have too few satellites.)  The Cholesky decomposition becomes
   * even more important for unscented filters. */
  int res;
  u32 i, j, k, cols = n*2;
  double m[n*cols];

  /* For now, we special-case only small matrices.  If we bring back
   * multiple antennas, it won't be hard to auto-generate cases 5 and
   * 6 if hard-coded routines prove more efficient. */
  switch (n) {
    case 2:
      return inv2(a, b);
      break;
    case 3:
      return inv3(a, b);
      break;
    case 4:
      return inv4(a, b);
      break;
    default:
      /* Set up an augmented matrix M = [A I] */
      for (i = 0; i < n; i++) {
        for (j = 0; j < cols; j++) {
          if (j >= n) {
            if (j-n == i) {
              m[i*cols+j] = 1.0;
            } else {
              m[i*cols+j] = 0;
            }
          } else {
            m[i*cols+j] = a[i*n+j];
          }
        }
      }

      if ((res = rref(n, cols, m)) < 0) {
        /* Singular matrix! */
        return res;
      }

      /* Extract B from the augmented matrix M = [I inv(A)] */
      for (i = 0; i < n; i++) {
        for (j = n, k = 0; j < cols; j++, k++) {
          b[i*n+k] = m[i*cols+j];
        }
      }

      return 0;
      break;
  }
}

/** Invert a non-square matrix (least-squares or least-norm solution).
 *  If \f$ A \f$ is of full rank, calculate the Moore-Penrose
 *  pseudoinverse \f$ A^{+} \f$ of a square matrix:
 *
 *  \f[ A \in \mathbb{R}^{n \times m} \f]
 *  \f[ B := A^{+} = \begin{cases}
 *      (A^{T} A)^{-1} A^{T} & \text{if } n > m \\
 *      A^{T} (A A^{T})^{-1} & \text{if } m > n \\
 *      A^{-1} & \text{if } n = m
 *  \end{cases} \f]
 * 
 * If \f$ n > m \f$, then \f$ A \f$ must be of full column rank, and
 * \f$ A^{+} \f$ solves the linear least-squares (overconstrained)
 * problem: \f[ x' = A^{+} b = \underset{x}{min} \|Ax - b\|_{2} \f]
 *
 * If \f$ m > n \f$, then \f$ A \f$ must be of full row rank, and \f$
 * A^{+} \f$ solves the linear least-norm (underconstrained) problem:
 * \f[ x' = A^{+} b = \underset{x}{min} \|x\|_{2} \text{s.t. } Ax = b
 * \f]
 *
 * If \f$ m = n \f$, then \f$ A \f$ must be of full rank, and \f$
 * A^{+} = A^{-1} \f$.
 *
 *  \param n            The number of rows in a
 *  \param m            The number of columns in a
 *  \param a            The matrix to invert (input)
 *  \param b            Where to put the inverse (output)
 *
 *  \return     -1 if a is singular; 0 otherwise.
 */
int matrix_pseudoinverse(u32 n, u32 m, const double *a, double *b) {
  if (n == m)
    return matrix_inverse(n, a, b);
  if (n > m)
    return matrix_ataiat(n, m, a, b);
  if (n < m)                                  /* n < m */
    return matrix_ataati(n, m, a, b);
  return -1;
}

/** Compute \f$ B := (A^{T} W A)^{-1} A^{T} \f$.
 *  Compute \f$ B := (A^{T} W A)^{-1} A^{T} \f$, where \f$ A \f$ is a
 *  matrix on \f$\mathbb{R}^{n \times m}\f$, \f$ W \f$ is a diagonal
 *  weighting matrix on \f$\mathbb{R}^{n \times n}\f$ and \f$B\f$ is
 *  (therefore) a matrix on \f$\mathbb{R}^{m \times m}\f$, for \f$ n >
 *  m \f$.
 *
 *  \param n            Number of rows in a
 *  \param m            Number of columns in a and rows and columns in b
 *  \param a            Input matrix
 *  \param w            Diagonal vector of weighting matrix 
 *  \param b            Output matrix
 *
 *  \return     -1 if n <= m or singular; 0 otherwise
 */
inline int matrix_atwaiat(u32 n, u32 m, const double *a,
                          const double *w, double *b) {
  u32 i, j, k;
  double c[m*m], inv[m*m];
  /* Check to make sure we're doing the right operation */
  if (n <= m) return -1;

  /* The resulting matrix is symmetric, so compute both halves at
   * once */
  for (i = 0; i < m; i++)
    for (j = i; j < m; j++) {
      c[m*i + j] = 0;
      if (i == j) {
        /* If this is a diagonal element, just sum the squares of the
         * column of A weighted by the weighting vector. */
        for (k = 0; k < n; k++)
          c[m*i + j] += w[k]*a[m*k + j]*a[m*k + j];
      } else {
        /* Otherwise, assign both off-diagonal elements at once. */
        for (k = 0; k < n; k++)
          c[m*i + j] = c[m*j + i] = w[k]*a[m*k + j]*a[m*k + i];
        c[m*j + i] = c[m*i + j];
      }
    }
  if (matrix_inverse(m, c, inv) < 0) return -1;
  for (i = 0; i < m; i++)
    for (j = 0; j < n; j++) {
      b[n*i + j] = 0;
      for (k = 0; k < m; k++)
        b[n*i + j] += inv[n*i+k] * a[m*j + k];
    }
  return 0;
}

/** Compute \f$ B := A^T (A W A^{T})^{-1} \f$.
 *  Compute \f$ B := A^T (A W A^{T})^{-1} \f$, where \f$ A \f$ is a
 *  matrix on \f$\mathbb{R}^{n \times m}\f$, \f$ W \f$ is a diagonal
 *  weighting matrix on \f$\mathbb{R}^{m \times m}\f$ and \f$B\f$ is
 *  (therefore) a matrix on \f$\mathbb{R}^{n \times n}\f$, for \f$ n <
 *  m \f$.
 *
 *  \param n            Number of rows in a and rows and columns in b
 *  \param m            Number of columns in a
 *  \param a            Input matrix
 *  \param w            Diagonal vector of weighting matrix 
 *  \param b            Output matrix
 *
 *  \return     -1 if n <= m or singular; 0 otherwise
 */
inline int matrix_atawati(u32 n, u32 m, const double *a,
                          const double *w, double *b) {
  u32 i, j, k;
  double c[m*m], inv[m*m];
  /* Check to make sure we're doing the right operation */
  if (n <= m) return -1;

  /* TODO(MP) -- implement! */
  /* The resulting matrix is symmetric, so compute both halves at
   * once */
  for (i = 0; i < m; i++)
    for (j = i; j < m; j++) {
      c[m*i + j] = 0;
      if (i == j) {
        /* If this is a diagonal element, just sum the squares of the
         * column of A weighted by the weighting vector. */
        for (k = 0; k < n; k++)
          c[m*i + j] += w[k]*a[m*k + j]*a[m*k + j];
      } else {
        /* Otherwise, assign both off-diagonal elements at once. */
        for (k = 0; k < n; k++)
          c[m*i + j] = c[m*j + i] = w[k]*a[m*k + j]*a[m*k + i];
        c[m*j + i] = c[m*i + j];
      }
    }
  if (matrix_inverse(m, c, inv) < 0) return -1;
  for (i = 0; i < m; i++)
    for (j = 0; j < n; j++) {
      b[n*i + j] = 0;
      for (k = 0; k < m; k++)
        b[n*i + j] += inv[n*i+k] * a[m*j + k];
    }
  return 0;
}

/** Compute \f$ B := (A^{T} A)^{-1} A^{T} \f$.
 *  Compute \f$ B := (A^{T} A)^{-1} A^{T} \f$, where \f$ A \f$ is a
 *  matrix on \f$\mathbb{R}^{n \times m}\f$ and \f$B\f$ is (therefore)
 *  a matrix on \f$\mathbb{R}^{m \times m}\f$, for \f$ n > m \f$.
 *
 *  \param n            Number of rows in a
 *  \param m            Number of columns in a and rows and columns in b
 *  \param a            Input matrix
 *  \param b            Output matrix
 *
 *  \return     -1 if n < m; 0 otherwise
 */
inline int matrix_ataiat(u32 n, u32 m, const double *a, double *b) {
  u32 i;
  double w[n];
  for (i = 0; i < n; i++) w[i] = 1;
  return matrix_atwaiat(n, m, a, w, b);
}

/** Compute \f$ B := A^{T} (A A^{T})^{-1} \f$.
 *  Compute \f$ B := A^{T} (A A^{T})^{-1} \f$, where \f$ A \f$ is a
 *  matrix on \f$\mathbb{R}^{n \times m}\f$ and \f$B\f$ is (therefore)
 *  a matrix on \f$\mathbb{R}^{n \times n}\f$, for \f$ n < m \f$.
 *
 *  \param n            Number of rows in a and rows and columns in b
 *  \param m            Number of columns in a
 *  \param a            Input matrix
 *  \param b            Output matrix
 *
 *  \return     -1 if n >= m or singular; 0 otherwise
 */
inline int matrix_ataati(u32 n, u32 m, const double *a, double *b) {
  u32 i;
  double w[n];
  for (i = 0; i < n; i++) w[i] = 1;
  return matrix_atawati(n, m, a, w, b);
}

/** Multiply two matrices.
 *  Multiply two matrices: \f$ C := AB \f$, where \f$ A \f$ is a
 *  matrix on \f$\mathbb{R}^{n \times m}\f$, \f$B\f$ is a matrix on
 *  \f$\mathbb{R}^{m \times p}\f$ and \f$C\f$ is (therefore) a matrix
 *  in \f$\mathbb{R}^{n \times p}\f$.
 *
 *  \param n            Number of rows in a and c
 *  \param m            Number of columns in a and rows in b
 *  \param p            Number of columns in b and c
 *  \param a            First matrix to multiply
 *  \param b            Second matrix to multiply
 *  \param c            Output matrix
 */
inline void matrix_multiply(u32 n, u32 m, u32 p, const double *a,
                            const double *b, double *c) {
  u32 i, j, k;
  for (i = 0; i < n; i++)
    for (j = 0; j < p; j++) {
      c[p*i + j] = 0;
      for (k = 0; k < m; k++)
        c[p*i + j] += a[m*i+k] * b[p*k + j];
    }
}

/** Add a matrix to a scaled matrix.
 *  Add two matrices: \f$ C := A + \gamma B \f$, where \f$ A \f$, \f$
 *  B \f$ and \f$C\f$ are matrices on \f$\mathbb{R}^{n \times m}\f$
 *  and \f$\gamma\f$ is a scalar coefficient.
 *
 *  \param n            Number of rows in a, b and c
 *  \param m            Number of columns in a, b and c
 *  \param a            First matrix (unscaled)
 *  \param b            Second matrix (will be scaled)
 *  \param gamma        Coefficient for second matrix
 *  \param c            Output (sum) matrix
 */
void matrix_add_sc(u32 n, u32 m, const double *a,
                   const double *b, double gamma, double *c) {
  u32 i, j;
  for (i = 0; i < n; i++)
    for (j = 0; j < m; j++)
      c[m*i + j] = a[m*i + j] + gamma * b[m*i + j];
}

/** Transpose a matrix.
 *  Transpose a matrix: \f$ B := A^{T} \f$, where \f$A\f$ is a matrix
 *  on \f$\mathbb{R}^{n \times m}\f$ and \f$B\f$ is (therefore) a
 *  matrix on \f$ \mathbb{R}^{m \times n}\f$.
 *
 *  \param n            Number of rows in \f$A\f$ and columns in \f$B\f$
 *  \param m            Number of rows in \f$B\f$ and columns in \f$A\f$
 *  \param a            Matrix to transpose
 *  \param b            Transposed (output) matrix
 */
void matrix_transpose(u32 n, u32 m,
                      const double *a, double *b) {
  u32 i, j;
  for (i = 0; i < n; i++)
    for (j = 0; j < m; j++)
      b[n*j+i] = a[m*i+j];
}

/** Copy a matrix.
 *  Copy a matrix: \f$ B := A \f$, where \f$A\f$ and \f$B\f$ are
 *  matrices on \f$\mathbb{R}^{n \times m}\f$.
 *
 *  \param n            Number of rows in \f$A\f$ and \f$B\f$
 *  \param m            Number of columns in \f$A\f$ and \f$B\f$
 *  \param a            Matrix to copy
 *  \param b            Copied (output) matrix
 */
void matrix_copy(u32 n, u32 m, const double *a,
                 double *b) {
  u32 i, j;
  for (i = 0; i < n; i++)
    for (j = 0; j < m; j++)
      b[m*i+j] = a[m*i+j];
}

/* \} */

/** \defgroup vectors Vector Mathematics
 *  Vector math routines.
 * \{ */

/** Dot product between two vectors.
 *  Compute the inner (scalar) (dot) product \f$ \vec{a} \cdot \vec{b}
 *  \f$, where \f$\vec{a}\f$ and \f$\vec{b}\f$ are vectors of length
 *  \f$n\f$.
 *
 *  \param n            Length of a and b
 *  \param a            First vector
 *  \param b            Second vector
 *
 *  \return     The dot product.
 */
double vector_dot(u32 n, const double *a,
                  const double *b) {
  u32 i;
  double out = 0;
  for (i = 0; i < n; i++)
    out += a[i]*b[i];
  return out;
}

/** Find the 2-norm of a vector.
 *  Compute the \f$l_{2}\f$ norm of a vector \f$\vec{a}\f$ of length
 *  \f$n\f$.
 *
 *  \param n            Length of a
 *  \param a            Input vector
 *
 *  \return     The 2-norm of a
 */
double vector_norm(u32 n, const double *a) {
  u32 i;
  double out = 0;
  for (i = 0; i < n; i++)
    out += a[i]*a[i];
  return sqrt(out);
}

/** Find the mean of a vector.
 *  Compute the mean of a vector \f$\vec{a}\f$ of length \f$n\f$.
 *
 *  \param n            Length of a
 *  \param a            Input vector
 *
 *  \return     The mean of \f$\vec{a}\f$
 */
double vector_mean(u32 n, const double *a) {
  u32 i;
  double out = 0;
  for (i = 0; i < n; i++)
    out += a[i];
  return out / n;
}

/** Normalize a vector in place.
 *  Rescale \f$\vec{a}\f$ of length \f$n\f$ so that its \f$ l_{2} \f$
 *  norm is equal to 1.
 *
 *  \param n            Length of a
 *  \param a            Vector to normalize
 */
void vector_normalize(u32 n, double *a) {
  u32 i;
  double norm = vector_norm(n, a);
  for (i = 0; i < n; i++)
    a[i] /= norm;
}

/** Add a vector with a scaled vector.
 *  Compute \f$ \vec{c} := \vec{a} + \gamma\vec{b} \f$, where
 *  \f$\vec{a}\f$, \f$\vec{b}\f$ and \f$\vec{c}\f$ are vectors of
 *  length \f$n\f$ and \f$\gamma\f$ is a scalar.
 *
 *  \param n            Length of a, b and c
 *  \param a            First input vector (unscaled)
 *  \param b            Second input vector (will be scaled)
 *  \param gamma        Coefficient for b
 *  \param c            Output vector
 */
void vector_add_sc(u32 n, const double *a,
                   const double *b, double gamma,
                   double *c) {
  u32 i;
  for (i = 0; i < n; i++)
    c[i] = a[i] + gamma * b[i];
}

/** Add two vectors.
 *  Compute \f$ \vec{c} := \vec{a} + \vec{b} \f$, where \f$\vec{a}\f$,
 *  \f$\vec{b}\f$ and \f$\vec{c}\f$ are vectors of length \f$n\f$.
 *
 *  \param n            Length of a, b and c
 *  \param a            First input vector
 *  \param b            Second input vector
 *  \param c            Output vector
 */
void vector_add(u32 n, const double *a,
                const double *b, double *c) {
  vector_add_sc(n, a, b, 1, c);
}

/** Subtract one vector from another.
 *  Compute \f$ \vec{c} := \vec{a} - \vec{b} \f$, where \f$\vec{a}\f$,
 *  \f$\vec{b}\f$ and \f$\vec{c}\f$ are vectors of length \f$n\f$.
 *
 *  \param n            Length of a, b and c
 *  \param a            First input vector
 *  \param b            Second input vector
 *  \param c            Output vector
 */
void vector_subtract(u32 n, const double *a,
                     const double *b, double *c) {
  vector_add_sc(n, a, b, -1, c);
}

/** Cross product of two vectors.
 * Compute \f$ \vec{c} := \vec{a} * \times \vec{b} \f$, where
 * \f$\vec{a}\f$, \f$\vec{b}\f$ and * \f$\vec{c}\f$ are 3-vectors.
 *
 *  \param n            Length of a, b and c
 *  \param a            First input vector
 *  \param b            Second input vector
 *  \param c            Output vector
 */
void vector_cross(const double a[3], const double b[3],
                  double c[3]) {
  c[0] = a[1]*b[2] - a[2]*b[1];
  c[1] = a[2]*b[0] - a[0]*b[2];
  c[2] = a[0]*b[1] - a[1]*b[0];
}

/* \} */


/*
 * Copyright (c) 2008, Morgan Quigley, Pieter Abbeel and Scott Gleason
 * Copyright (c) 2010, Matt Peddie
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

/* TODO: this file needs some love. */

#include <math.h>
#include "linear_algebra.h"

/* Ainv := A^(-1) */
void invert4x4(const double const A[4][4], double Ainv[4][4])
{
  double detA = A[0][0]*A[1][1]*A[2][2]*A[3][3]-A[0][0]*A[1][1]*A[2][3]*A[3][2]-A[0][0]*A[2][1]*A[1][2]*A[3][3]+A[0][0]*A[2][1]*A[1][3]*A[3][2]+A[0][0]*A[3][1]*A[1][2]*A[2][3]-A[0][0]*A[3][1]*A[1][3]*A[2][2]-A[1][0]*A[0][1]*A[2][2]*A[3][3]+A[1][0]*A[0][1]*A[2][3]*A[3][2]+A[1][0]*A[2][1]*A[0][2]*A[3][3]-A[1][0]*A[2][1]*A[0][3]*A[3][2]-A[1][0]*A[3][1]*A[0][2]*A[2][3]+A[1][0]*A[3][1]*A[0][3]*A[2][2]+A[2][0]*A[0][1]*A[1][2]*A[3][3]-A[2][0]*A[0][1]*A[1][3]*A[3][2]-A[2][0]*A[1][1]*A[0][2]*A[3][3]+A[2][0]*A[1][1]*A[0][3]*A[3][2]+A[2][0]*A[3][1]*A[0][2]*A[1][3]-A[2][0]*A[3][1]*A[0][3]*A[1][2]-A[3][0]*A[0][1]*A[1][2]*A[2][3]+A[3][0]*A[0][1]*A[1][3]*A[2][2]+A[3][0]*A[1][1]*A[0][2]*A[2][3]-A[3][0]*A[1][1]*A[0][3]*A[2][2]-A[3][0]*A[2][1]*A[0][2]*A[1][3]+A[3][0]*A[2][1]*A[0][3]*A[1][2];

  Ainv[0][0] = (A[1][1]*A[2][2]*A[3][3]-A[1][1]*A[2][3]*A[3][2]-A[2][1]*A[1][2]*A[3][3]+A[2][1]*A[1][3]*A[3][2]+A[3][1]*A[1][2]*A[2][3]-A[3][1]*A[1][3]*A[2][2])/detA;
  Ainv[0][1] = (-A[0][1]*A[2][2]*A[3][3]+A[0][1]*A[2][3]*A[3][2]+A[2][1]*A[0][2]*A[3][3]-A[2][1]*A[0][3]*A[3][2]-A[3][1]*A[0][2]*A[2][3]+A[3][1]*A[0][3]*A[2][2])/detA;
  Ainv[0][2] = (A[0][1]*A[1][2]*A[3][3]-A[0][1]*A[1][3]*A[3][2]-A[1][1]*A[0][2]*A[3][3]+A[1][1]*A[0][3]*A[3][2]+A[3][1]*A[0][2]*A[1][3]-A[3][1]*A[0][3]*A[1][2])/detA;
  Ainv[0][3] = (-A[0][1]*A[1][2]*A[2][3]+A[0][1]*A[1][3]*A[2][2]+A[1][1]*A[0][2]*A[2][3]-A[1][1]*A[0][3]*A[2][2]-A[2][1]*A[0][2]*A[1][3]+A[2][1]*A[0][3]*A[1][2])/detA;

  Ainv[1][0] = (-A[1][0]*A[2][2]*A[3][3]+A[1][0]*A[2][3]*A[3][2]+A[2][0]*A[1][2]*A[3][3]-A[2][0]*A[1][3]*A[3][2]-A[3][0]*A[1][2]*A[2][3]+A[3][0]*A[1][3]*A[2][2])/detA;
  Ainv[1][1] = (A[0][0]*A[2][2]*A[3][3]-A[0][0]*A[2][3]*A[3][2]-A[2][0]*A[0][2]*A[3][3]+A[2][0]*A[0][3]*A[3][2]+A[3][0]*A[0][2]*A[2][3]-A[3][0]*A[0][3]*A[2][2])/detA;
  Ainv[1][2] = (-A[0][0]*A[1][2]*A[3][3]+A[0][0]*A[1][3]*A[3][2]+A[1][0]*A[0][2]*A[3][3]-A[1][0]*A[0][3]*A[3][2]-A[3][0]*A[0][2]*A[1][3]+A[3][0]*A[0][3]*A[1][2])/detA;
  Ainv[1][3] = (A[0][0]*A[1][2]*A[2][3]-A[0][0]*A[1][3]*A[2][2]-A[1][0]*A[0][2]*A[2][3]+A[1][0]*A[0][3]*A[2][2]+A[2][0]*A[0][2]*A[1][3]-A[2][0]*A[0][3]*A[1][2])/detA;

  Ainv[2][0] = (A[1][0]*A[2][1]*A[3][3]-A[1][0]*A[3][1]*A[2][3]-A[2][0]*A[1][1]*A[3][3]+A[2][0]*A[3][1]*A[1][3]+A[3][0]*A[1][1]*A[2][3]-A[3][0]*A[2][1]*A[1][3])/detA;
  Ainv[2][1] = (-A[0][0]*A[2][1]*A[3][3]+A[0][0]*A[3][1]*A[2][3]+A[2][0]*A[0][1]*A[3][3]-A[2][0]*A[0][3]*A[3][1]-A[3][0]*A[0][1]*A[2][3]+A[3][0]*A[0][3]*A[2][1])/detA;
  Ainv[2][2] = (A[0][0]*A[1][1]*A[3][3]-A[0][0]*A[3][1]*A[1][3]-A[1][0]*A[0][1]*A[3][3]+A[1][0]*A[0][3]*A[3][1]+A[3][0]*A[0][1]*A[1][3]-A[3][0]*A[0][3]*A[1][1])/detA;
  Ainv[2][3] = (-A[0][0]*A[1][1]*A[2][3]+A[0][0]*A[2][1]*A[1][3]+A[1][0]*A[0][1]*A[2][3]-A[1][0]*A[0][3]*A[2][1]-A[2][0]*A[0][1]*A[1][3]+A[2][0]*A[0][3]*A[1][1])/detA;

  Ainv[3][0] = (-A[1][0]*A[2][1]*A[3][2]+A[1][0]*A[3][1]*A[2][2]+A[2][0]*A[1][1]*A[3][2]-A[2][0]*A[3][1]*A[1][2]-A[3][0]*A[1][1]*A[2][2]+A[3][0]*A[2][1]*A[1][2])/detA;
  Ainv[3][1] = (A[0][0]*A[2][1]*A[3][2]-A[0][0]*A[3][1]*A[2][2]-A[2][0]*A[0][1]*A[3][2]+A[2][0]*A[0][2]*A[3][1]+A[3][0]*A[0][1]*A[2][2]-A[3][0]*A[0][2]*A[2][1])/detA;
  Ainv[3][2] = (-A[0][0]*A[1][1]*A[3][2]+A[0][0]*A[3][1]*A[1][2]+A[1][0]*A[0][1]*A[3][2]-A[1][0]*A[0][2]*A[3][1]-A[3][0]*A[0][1]*A[1][2]+A[3][0]*A[0][2]*A[1][1])/detA;
  Ainv[3][3] = (A[0][0]*A[1][1]*A[2][2]-A[0][0]*A[2][1]*A[1][2]-A[1][0]*A[0][1]*A[2][2]+A[1][0]*A[0][2]*A[2][1]+A[2][0]*A[0][1]*A[1][2]-A[2][0]*A[0][2]*A[1][1])/detA;

}

/* return sqrt(v_1^2 + v_2^2 + v_3^2) */
double vector_norm(const double const v[3])
{
  return(sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]));
}

/* v := v / ||v|| */
void vector_normalise(double v[3])
{
  double norm = vector_norm(v);
  v[0] /= norm;
  v[1] /= norm;
  v[2] /= norm;
}

/* n = sqrt(v_1^2 + v_2^2 + . . . + v_k^2) */
double vector_norm_n(u32 n, const double const v[])
{
  double sum = 0;

  for(u32 i=0; i<n; i++) {
    sum += v[i]*v[i];
  }
  return sqrt(sum);
}

/* n = v_1 + v_2 + . . . + v_k */
double vector_mean_n(u32 n, const double const v[])
{
  double sum = 0;

  for(u32 i=0; i<n; i++) {
    sum += v[i];
  }
  return sum/n;
}

/* C := A x B */
void vector_cross(const double const a[3], const double const b[3], double c[3])
{
  c[0] = a[1]*b[2] - a[2]*b[1];
  c[1] = a[2]*b[0] - a[0]*b[2];
  c[2] = a[0]*b[1] - a[1]*b[0];
}

/* C := A + B       A, B, C e R^3 */
void vector_add(const double const a[3], const double const b[3], double c[3])
{
  c[0] = a[0] + b[0];
  c[1] = a[1] + b[1];
  c[2] = a[2] + b[2];
}

/* C := A + B       A, B, C e R^n */
void vector_add_n(const double const a[], const double const b[], double c[], u32 n)
{
  for (u32 i=0; i<n; i++) {
    c[i] = a[i] + b[i];
  }
}

/* C := A - B       A, B, C e R^3 */
void vector_subtract(const double const a[3], const double const b[3], double c[3])
{
  c[0] = a[0] - b[0];
  c[1] = a[1] - b[1];
  c[2] = a[2] - b[2];
}

/* c := Dv        c, v e R^n   D e R^(n x n) = δ * d e R^n */
void diag_matrix_vector_multiply(u32 n, double c[n], const double const D[n], const double const v[n]) {
  for (u32 i=0; i<n; i++) {
    c[i] = D[i]*v[i];
  }
}

/* A := B */
void matrix_copy(double *A, unsigned rows, unsigned columns, const double const *B)
{
  for(u32 i=0; i<rows*columns; i++) {
    A[i] = B[i];
  }
}

/********************************************************
 * Parameters:
 *           rowsA - number of rows of A
 *           columnsA - number of columns of A
 *           columnsB - number of columns of B
 *           A - pointer to a rowsA x columnsA matrix
 *           B - pointer to a columnsA x columnsB matrix
 *           C - pointer to a rowsA x columnsB matrix
 *********************************************************/
void matrix_multiply(unsigned rowsA, unsigned columnsA, unsigned columnsB, const double* const A, const double* const B, double *C)
{
  unsigned i, j, k;

  for(i = 0;i < rowsA;i++){

    for(j = 0;j < columnsB;j++){

      C[i*columnsB +j] = 0.0;

      for(k = 0;k < columnsA;k++){
        C[i*columnsB +j] +=  A[i*columnsA+k]*B[k*columnsB+j];
      }
    }
  }
}

/********************************************************
 * Parameters:
 *           rowsA - number of rows of A = columns of diagonal matrix A
 *           columnsB - number of columns of B
 *           A - pointer to diagonal element vector
 *           B - pointer to a rowsA x columnsB matrix
 *           C - pointer to a rowsA x columnsB matrix
 *********************************************************/
void diag_matrix_multiply(unsigned rowsA, unsigned columnsB, const double* const A, const double const *B, double *C)
{
  unsigned i, j;

  for (i = 0; i < rowsA; i++) {

    for (j = 0; j < columnsB; j++) {

      C[i * columnsB + j] = A[i] * B[i * columnsB + j];
    }
  }
}



/********************************************************
 * Parameters:
 *           rows - number of rows of A
 *           columns - number of columns of A
 *           A - pointer to a rows x columns matrix
 *           B - pointer to a columns x rows matrix, transpose(A)
 *********************************************************/
void matrix_transpose(unsigned rows, unsigned columns, const double const *A, double *B)
{
  unsigned i, j;

  for(i = 0;i < columns;i++){
    for(j = 0;j < rows;j++){
      B[i*rows+j] = A[j*columns+i];
    }
  }
}

/* return a . b */
double vector_dot_product (const double const *a, const double const *b)
{
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

/* Apparently there is no C code online that simply inverts a general
 * matrix without having to include GSL or something.  Therefore, I
 * give you Gaussian Elimination for matrix inversion.  --MP */
#define MATRIX_EPSILON (1e-22)

static void row_swap(double *a, double *b, u32 size)
{
  double tmp;
  for(u32 i=0; i<size; i++) {
    tmp = a[i];
    a[i] = b[i];
    b[i] = tmp;
  }
}

static int rref(u32 order, u32 cols, double *m)
{
  int i, j, k, maxrow;
  double tmp;

  for (i=0; i<(int) order; i++) {
    maxrow = i;
    for (j=i+1; j<(int) order; j++) { 
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
    for (j=i+1; j<(int) order; j++) {
      /* Elimination of column i */
      tmp = m[j*cols+i] / m[i*cols+i];
      for (k=i; k<(int) cols; k++) {
        m[j*cols+k] -= m[i*cols+k] * tmp;
      }
    }
  }
  for (i=order-1; i>=0; i--) {
    /* Back-substitution */
    tmp = m[i*cols+i];
    if (tmp == 0) {
      /*WARNING("Solution is going to go bonkers; tmp = 0 line 237 of lin_alg.c; make Matt fix it.\n");*/
      return -1;
    }
    for (j=0; j<i; j++) {
      for (k=cols-1; k>i-1; k--) {
        m[j*cols+k] -= m[i*cols+k] * m[j*cols+i] / tmp;
      }
    }
    m[i*cols+i] /= tmp;
    for (j=order; j<(int) cols; j++) {
      /* Normalize row */
      m[i*cols+j] /= tmp;
    }
  }
  return 0;
}

int matrix_inverse(u32 order, const double const *a, double *b)
{
  int res;
  u32 i, j, k, cols = order*2;
  double m[order*cols];

  /* Set up an augmented matrix M = [A I] */
  for (i=0; i<order; i++) {
    for (j=0; j<cols; j++) {
      if(j>=order) {
        if (j-order == i) {
          m[i*cols+j] = 1.0; 
        } else m[i*cols+j] = 0;
      } else {
        m[i*cols+j] = a[i*order+j];
      }
    }
  }
  
  if ((res = rref(order, cols, m)) < 0) {
    /* printf("=================\nSingular matrix (zero diagonal after row reduction:\n"); */
    /* print_mat2(m, order, cols); */
    /* printf("=================\n\n"); */
    return res;
  }

  /* Extract B from the augmented matrix M = [I inv(A)] */
  for (i=0; i<order; i++) {
    for (j=order, k=0; j<cols; j++,k++) {
      b[i*order+k] = m[i*cols+j];
    }
  }

  /* print_mat(b, order); */
  return 0;
}


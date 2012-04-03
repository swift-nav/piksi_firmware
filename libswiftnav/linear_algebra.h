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

#ifndef SWIFTLIB_LINEAR_ALGEBRA_H
#define SWIFTLIB_LINEAR_ALGEBRA_H

#include "common.h"

int matrix_inverse(u32 n, const double const *a, double *b);
void matrix_multiply(u32 n, u32 m, u32 p, const double *a,
                     const double *b, double *c);
void matrix_add_sc(u32 n, u32 m, const double *a,
                   const double *b, double gamma, double *c);
void matrix_transpose(u32 n, u32 m, const double *a, double *b);
void matrix_copy(u32 n, u32 m, const double *a, double *b);

double vector_dot(u32 n, const double *a, const double *b);
double vector_norm(u32 n, const double *a);
double vector_mean(u32 n, const double *a);
void vector_normalize(u32 n, double *a);
void vector_add_sc(u32 n, const double *a, const double *b,
                   double gamma, double *c);
void vector_add(u32 n, const double *a, const double *b, double *c);
void vector_subtract(u32 n, const double *a,
                     const double *b, double *c);
void vector_cross(const double a[3], const double b[3], double c[3]);

#endif  /* SWIFTLIB_LINEAR_ALGEBRA_H */

/*
 * linear_algebra_new.h
 * Linear algebra routines for SwiftLib
 * Copyright (c) 2012, Matt Peddie <peddie@alum.mit.edu>
 */

#ifndef SWIFTLIB_COORD_SYSTEM_H
#define SWIFTLIB_COORD_SYSTEM_H

#include "common.h"

int matrix_inverse_new(u32 order, const double const *a, double *b);
void matrix_multiply_new(u32 n, u32 m, u32 p, const double *a,
                         const double *b, double *c);
void matrix_transpose_new(u32 n, u32 m, const double *a, double *b);
void matrix_add_sc(u32 n, u32 m, const double *a,
                   const double *b, double gamma, double *c);
void matrix_transpose_new(u32 n, u32 m, const double *a, double *b);
void matrix_copy_new(u32 n, u32 m, const double *a, double *b);

double vector_dot_new(u32 n, const double *a, const double *b);
double vector_norm_new(u32 n, const double *a);
double vector_mean_new(u32 n, const double *a);
void vector_normalize_new(u32 n, double *a);
void vector_add_sc(u32 n, const double *a, const double *b,
                   double gamma, double *c);
void vector_add_new(u32 n, const double *a, const double *b, double *c);
void vector_subtract_new(u32 n, const double *a,
                         const double *b, double *c);
void vector_cross_new(const double a[3], const double b[3], double c[3]);

#endif  /* SWIFTLIB_COORD_SYSTEM_H */

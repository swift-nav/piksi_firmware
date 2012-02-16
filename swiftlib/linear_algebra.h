/*
 * Copyright (c) 2008, Morgan Quigley, Pieter Abbeel and Scott Gleason
 * Copyright (c) 2010, C.O. Lee Boyce Jr.
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

#ifndef SWIFTLIB_LINEAR_ALGEBRA_H
#define SWIFTLIB_LINEAR_ALGEBRA_H

#include "common.h"

double vector_dot_product (const double const *a, const double const *b);
void vector_cross(const double const a[3], const double const b[3], double c[3]);
double vector_norm(const double const v[3]);
double vector_norm_n(u32 n, const double const v[]);
double vector_mean_n(u32 n, const double const v[]);
void vector_normalise(double v[]);
void vector_subtract(const double const a[3], const double const b[3], double c[3]); // c = a - b
void vector_add(const double const a[3], const double const b[3], double c[3]);
void vector_add_n(const double const a[], const double const b[], double c[], u32 n);

void invert4x4(const double const A[4][4], double Ainv[4][4]);
void matrix_multiply(unsigned N, unsigned M, unsigned L, const double* const a, const double* const b, double *c);
void matrix_transpose(unsigned rows, unsigned columns, const double* const A, double *B);
void matrix_copy(double *A, unsigned rows, unsigned columns, const double const *B);
void diag_matrix_multiply(unsigned rowsA, unsigned columnsB, const double* const A, const double* const B, double *C);
void diag_matrix_vector_multiply(u32 n, double c[n], const double const D[n], const double const v[n]);

int matrix_inverse(u32 order, const double const *a, double *b);

#endif

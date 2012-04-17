#include <math.h>
#include <stdlib.h>
#include <check.h>

#include <stdio.h>
#include "check_utils.h"

#include "../linear_algebra.h"

#define LINALG_TOL 1e-10
#define LINALG_NUM 222
#define MATRIX_MIN -1e22
#define MATRIX_MAX 1e22
#define mrand frand(MATRIX_MIN, MATRIX_MAX)
#define MSIZE_MAX 65536

/* TODO: matrix_multiply, matrix_add_sc, matrix_copy, all vector functions */

START_TEST(test_matrix_transpose) {
  unsigned int i, j, t;
  u32 n = random() / RAND_MAX * MSIZE_MAX;
  u32 m = random() / RAND_MAX * MSIZE_MAX;
  double A[n*m];
  double B[m*n];
  double C[n*m];

  seed_rng();
  for (t = 0; t < LINALG_NUM; t++) {
    for (i = 0; i < n; i++)
      for (j = 0; j < m; j++)
        A[n*i + j] = mrand;
    matrix_transpose(n, m, A, B);
    matrix_transpose(m, n, B, C);
    for (i = 0; i < n; i++)
      for (j = 0; j < m; j++)
        fail_unless(fabs(A[n*i + j] - C[n*i + j]) < LINALG_TOL);
  }
}
END_TEST

START_TEST(test_matrix_inverse_2x2) {
  unsigned int i, j, t;
  double A[4];
  double B[4];
  double I[4];

  seed_rng();
  /* 2x2 inverses */
  for (t = 0; t < LINALG_NUM; t++) {
    do {
      for (i = 0; i < 2; i++)
        for (j = 0; j < 2; j++)
          A[2*i + j] = mrand;
    } while (matrix_inverse(2, A, B) < 0);
    matrix_multiply(2, 2, 2, A, B, I);
    fail_unless(fabs(I[0] - 1) < LINALG_TOL,
                "Matrix differs from identity: %lf", I[0]);
    fail_unless(fabs(I[3] - 1) < LINALG_TOL,
                "Matrix differs from identity: %lf", I[3]);
  }
}
END_TEST

START_TEST(test_matrix_inverse_3x3) {
  unsigned int i, j, t;
  double A[9];
  double B[9];
  double I[9];

  seed_rng();
  /* 3x3 inverses */
  for (t = 0; t < LINALG_NUM; t++) {
    do {
      for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
          A[3*i + j] = mrand;
    } while (matrix_inverse(3, A, B) < 0);
    matrix_multiply(3, 3, 3, A, B, I);
    fail_unless(fabs(I[0] - 1) < LINALG_TOL,
                "Matrix differs from identity: %lf", I[0]);
    fail_unless(fabs(I[4] - 1) < LINALG_TOL,
                "Matrix differs from identity: %lf", I[4]);
    fail_unless(fabs(I[8] - 1) < LINALG_TOL,
                "Matrix differs from identity: %lf", I[8]);
  }
}
END_TEST

START_TEST(test_matrix_inverse_4x4) {
  unsigned int i, j, t;
  double A[16];
  double B[16];
  double I[16];

  seed_rng();
  /* 4x4 inverses */
  for (t = 0; t < LINALG_NUM; t++) {
    do {
      for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
          A[4*i + j] = mrand;
    } while (matrix_inverse(4, A, B) < 0);
    matrix_multiply(4, 4, 4, A, B, I);
    fail_unless(fabs(I[0] - 1) < LINALG_TOL,
                "Matrix differs from identity: %lf", I[0]);
    fail_unless(fabs(I[5] - 1) < LINALG_TOL,
                "Matrix differs from identity: %lf", I[5]);
    fail_unless(fabs(I[10] - 1) < LINALG_TOL,
                "Matrix differs from identity: %lf", I[10]);
    fail_unless(fabs(I[15] - 1) < LINALG_TOL,
                "Matrix differs from identity: %lf", I[15]);
  }
}
END_TEST

START_TEST(test_matrix_inverse_5x5) {
  unsigned int i, j, t;
  double A[25];
  double B[25];
  double I[25];

  seed_rng();
  /* 5x5 inverses */
  for (t = 0; t < LINALG_NUM; t++) {
    do {
      for (i = 0; i < 5; i++)
        for (j = 0; j < 5; j++)
          A[5*i + j] = mrand;
    } while (matrix_inverse(5, A, B) < 0);
    matrix_multiply(5, 5, 5, A, B, I);
    fail_unless(fabs(I[0] - 1) < LINALG_TOL,
                "Matrix differs from identity: %lf", I[0]);
    fail_unless(fabs(I[6] - 1) < LINALG_TOL,
                "Matrix differs from identity: %lf", I[6]);
    fail_unless(fabs(I[12] - 1) < LINALG_TOL,
                "Matrix differs from identity: %lf", I[12]);
    fail_unless(fabs(I[18] - 1) < LINALG_TOL,
                "Matrix differs from identity: %lf", I[18]);
    fail_unless(fabs(I[24] - 1) < LINALG_TOL,
                "Matrix differs from identity: %lf", I[24]);
  }
}
END_TEST


Suite* linear_algebra_suite(void) {
  Suite *s = suite_create("Linear algebra");

  /* Core test case */
  TCase *tc_core = tcase_create("Core");
  tcase_add_test(tc_core, test_matrix_transpose);
  tcase_add_test(tc_core, test_matrix_inverse_2x2);
  tcase_add_test(tc_core, test_matrix_inverse_3x3);
  tcase_add_test(tc_core, test_matrix_inverse_4x4);
  tcase_add_test(tc_core, test_matrix_inverse_5x5);
  suite_add_tcase(s, tc_core);

  return s;
}


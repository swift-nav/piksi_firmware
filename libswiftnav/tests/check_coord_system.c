#include <math.h>

#include <check.h>
#include "check_utils.h"

#include "../coord_system.h"
#include "../pvt.h"

/* Maximum allowable error in quantities with units of length (in meters). */
#define MAX_DIST_ERROR_M 1e-6
/* Maximum allowable error in quantities with units of angle (in sec of arc).
 * 1 second of arc on the equator is ~31 meters. */
#define MAX_ANGLE_ERROR_SEC 1e-7
#define MAX_ANGLE_ERROR_RAD (MAX_ANGLE_ERROR_SEC*(D2R/3600.0))

/* Semi-major axis. */
#define EARTH_A 6378137.0
/* Semi-minor axis. */
#define EARTH_B 6356752.31424517929553985595703125

#define NUM_COORDS 10
const double const llhs[NUM_COORDS][3] = {
  {0, 0, 0},        /* On the Equator and Prime Meridian. */
  {0, 180*D2R, 0},  /* On the Equator. */
  {0, 90*D2R, 0},   /* On the Equator. */
  {0, -90*D2R, 0},  /* On the Equator. */
  {90*D2R, 0, 0},   /* North pole. */
  {-90*D2R, 0, 0},  /* South pole. */
  {90*D2R, 0, 22},  /* 22m above the north pole. */
  {-90*D2R, 0, 22}, /* 22m above the south pole. */
  {0, 0, 22},       /* 22m above the Equator and Prime Meridian. */
  {0, 180*D2R, 22}, /* 22m above the Equator. */
};
const double const ecefs[NUM_COORDS][3] = {
  {EARTH_A, 0, 0},
  {-EARTH_A, 0, 0},
  {0, EARTH_A, 0},
  {0, -EARTH_A, 0},
  {0, 0, EARTH_B},
  {0, 0, -EARTH_B},
  {0, 0, (EARTH_B+22)},
  {0, 0, -(EARTH_B+22)},
  {(22+EARTH_A), 0, 0},
  {-(22+EARTH_A), 0, 0},
};

START_TEST(test_wgsllh2ecef)
{
  double ecef[3];

  wgsllh2ecef(llhs[_i], ecef);

  for (int n=0; n<3; n++) {
    fail_unless(!isnan(ecef[n]), "NaN in output from wgsllh2ecef.");
    double err = fabs(ecef[n] - ecefs[_i][n]);
    fail_unless(err < MAX_DIST_ERROR_M,
      "Conversion from WGS84 LLH to ECEF has >1e-6m error:\n"
      "LLH: %f, %f, %f\n"
      "X error (mm): %g\nY error (mm): %g\nZ error (mm): %g",
      llhs[_i][0]*R2D, llhs[_i][1]*R2D, llhs[_i][2],
      (ecef[0] - ecefs[_i][0])*1e3,
      (ecef[1] - ecefs[_i][1])*1e3,
      (ecef[2] - ecefs[_i][2])*1e3
    );
  }
}
END_TEST

START_TEST(test_wgsecef2llh)
{
  double llh[3];

  wgsecef2llh(ecefs[_i], llh);

  for (int n=0; n<3; n++) {
    fail_unless(!isnan(llh[n]), "NaN in output from wgsecef2llh.");
  }

  double lat_err = fabs(llh[0] - llhs[_i][0]);
  double lon_err = fabs(llh[1] - llhs[_i][1]);
  double hgt_err = fabs(llh[2] - llhs[_i][2]);
  fail_unless((lat_err < MAX_ANGLE_ERROR_RAD) ||
              (lon_err < MAX_ANGLE_ERROR_RAD) ||
              (hgt_err < MAX_DIST_ERROR_M),
    "Conversion from WGS84 ECEF to LLH has >1e-6 {rad, m} error:\n"
    "ECEF: %f, %f, %f\n"
    "Lat error (arc sec): %g\nLon error (arc sec): %g\nH error (mm): %g",
    ecefs[_i][0], ecefs[_i][1], ecefs[_i][2],
    (llh[0] - llhs[_i][0])*(R2D*3600),
    (llh[1] - llhs[_i][1])*(R2D*3600),
    (llh[2] - llhs[_i][2])*1e3
  );
}
END_TEST

START_TEST(test_wgsllh2ecef2llh)
{
  double ecef[3];
  double llh[3];

  wgsllh2ecef(llhs[_i], ecef);
  wgsecef2llh(ecef, llh);

  for (int n=0; n<3; n++) {
    fail_unless(!isnan(llh[n]), "NaN in LLH output from conversion.");
    fail_unless(!isnan(ecef[n]), "NaN in ECEF output from conversion.");
  }

  double lat_err = fabs(llh[0] - llhs[_i][0]);
  double lon_err = fabs(llh[1] - llhs[_i][1]);
  double hgt_err = fabs(llh[2] - llhs[_i][2]);
  fail_unless((lat_err < MAX_ANGLE_ERROR_RAD) ||
              (lon_err < MAX_ANGLE_ERROR_RAD) ||
              (hgt_err < MAX_DIST_ERROR_M),
    "Converting WGS84 LLH to ECEF and back again does not return the "
    "original values.\n"
    "Initial LLH: %f, %f, %f\n"
    "ECEF: %f, %f, %f\n"
    "Final LLH: %f, %f, %f\n"
    "Lat error (arc sec): %g\nLon error (arc sec): %g\nH error (mm): %g",
    R2D*llhs[_i][0], R2D*llhs[_i][1], llhs[_i][2],
    ecef[0], ecef[1], ecef[2],
    R2D*llh[0], R2D*llh[1], llh[2],
    (llh[0] - llhs[_i][0])*(R2D*3600),
    (llh[1] - llhs[_i][1])*(R2D*3600),
    (llh[2] - llhs[_i][2])*1e3
  );
}
END_TEST

START_TEST(test_wgsecef2llh2ecef)
{
  double llh[3];
  double ecef[3];

  wgsecef2llh(ecefs[_i], llh);
  wgsllh2ecef(llh, ecef);

  for (int n=0; n<3; n++) {
    fail_unless(!isnan(llh[n]), "NaN in LLH output from conversion.");
    fail_unless(!isnan(ecef[n]), "NaN in ECEF output from conversion.");
  }

  for (int n=0; n<3; n++) {
    double err = fabs(ecef[n] - ecefs[_i][n]);
    fail_unless(err < MAX_DIST_ERROR_M,
      "Converting WGS84 ECEF to LLH and back again does not return the "
      "original values.\n"
      "Initial ECEF: %f, %f, %f\n"
      "LLH: %f, %f, %f\n"
      "Final ECEF: %f, %f, %f\n"
      "X error (mm): %g\nY error (mm): %g\nZ error (mm): %g",
      ecefs[_i][0], ecefs[_i][1], ecefs[_i][2],
      R2D*llh[0], R2D*llh[1], llh[2],
      ecef[0], ecef[1], ecef[2],
      (ecef[0] - ecefs[_i][0])*1e3,
      (ecef[1] - ecefs[_i][1])*1e3,
      (ecef[2] - ecefs[_i][2])*1e3
    );
  }
}
END_TEST

START_TEST(test_random_wgsllh2ecef2llh)
{
  double ecef[3];
  double llh_init[3];
  double llh[3];

  seed_rng();

  llh_init[0] = D2R*frand(-90, 90);
  llh_init[1] = D2R*frand(-180, 180);
  llh_init[2] = frand(-EARTH_A, 4*EARTH_A);

  wgsllh2ecef(llh_init, ecef);
  wgsecef2llh(ecef, llh);

  for (int n=0; n<3; n++) {
    fail_unless(!isnan(llh[n]), "NaN in LLH output from conversion.");
    fail_unless(!isnan(ecef[n]), "NaN in ECEF output from conversion.");
  }

  double lat_err = fabs(llh[0] - llh_init[0]);
  double lon_err = fabs(llh[1] - llh_init[1]);
  double hgt_err = fabs(llh[2] - llh_init[2]);
  fail_unless((lat_err < MAX_ANGLE_ERROR_RAD) ||
              (lon_err < MAX_ANGLE_ERROR_RAD) ||
              (hgt_err < MAX_DIST_ERROR_M),
    "Converting random WGS84 LLH to ECEF and back again does not return the "
    "original values.\n"
    "Initial LLH: %f, %f, %f\n"
    "ECEF: %f, %f, %f\n"
    "Final LLH: %f, %f, %f\n"
    "Lat error (arc sec): %g\nLon error (arc sec): %g\nH error (mm): %g",
    R2D*llh_init[0], R2D*llh_init[1], llh_init[2],
    ecef[0], ecef[1], ecef[2],
    R2D*llh[0], R2D*llh[1], llh[2],
    (llh[0] - llh_init[0])*(R2D*3600),
    (llh[1] - llh_init[1])*(R2D*3600),
    (llh[2] - llh_init[2])*1e3
  );

  fail_unless((R2D*llh[0] >= -90) && (R2D*llh[0] <= 90),
      "Converting random WGS86 ECEF gives latitude out of bounds.\n"
      "ECEF: %f, %f, %f\n"
      "LLH: %f, %f, %f\n",
      ecef[0], ecef[1], ecef[2],
      R2D*llh[0], R2D*llh[1], llh[2]
  );

  fail_unless((R2D*llh[1] >= -180) && (R2D*llh[1] <= 180),
      "Converting random WGS86 ECEF gives longitude out of bounds.\n"
      "ECEF: %f, %f, %f\n"
      "LLH: %f, %f, %f\n",
      ecef[0], ecef[1], ecef[2],
      R2D*llh[0], R2D*llh[1], llh[2]
  );

  fail_unless(llh[2] > -EARTH_A,
      "Converting random WGS86 ECEF gives height out of bounds.\n"
      "ECEF: %f, %f, %f\n"
      "LLH: %f, %f, %f\n",
      ecef[0], ecef[1], ecef[2],
      R2D*llh[0], R2D*llh[1], llh[2]
  );
}
END_TEST

START_TEST(test_random_wgsecef2llh2ecef)
{
  double ecef_init[3];
  double llh[3];
  double ecef[3];

  seed_rng();

  ecef_init[0] = frand(-4*EARTH_A, 4*EARTH_A);
  ecef_init[1] = frand(-4*EARTH_A, 4*EARTH_A);
  ecef_init[2] = frand(-4*EARTH_A, 4*EARTH_A);

  wgsecef2llh(ecef_init, llh);
  wgsllh2ecef(llh, ecef);

  for (int n=0; n<3; n++) {
    fail_unless(!isnan(llh[n]), "NaN in LLH output from conversion.");
    fail_unless(!isnan(ecef[n]), "NaN in ECEF output from conversion.");
  }

  for (int n=0; n<3; n++) {
    double err = fabs(ecef[n] - ecef_init[n]);
    fail_unless(err < MAX_DIST_ERROR_M,
      "Converting random WGS84 ECEF to LLH and back again does not return the "
      "original values.\n"
      "Initial ECEF: %f, %f, %f\n"
      "LLH: %f, %f, %f\n"
      "Final ECEF: %f, %f, %f\n"
      "X error (mm): %g\nY error (mm): %g\nZ error (mm): %g",
      ecef_init[0], ecef_init[1], ecef_init[2],
      R2D*llh[0], R2D*llh[1], llh[2],
      ecef[0], ecef[1], ecef[2],
      (ecef[0] - ecef_init[0])*1e3,
      (ecef[1] - ecef_init[1])*1e3,
      (ecef[2] - ecef_init[2])*1e3
    );
  }

  fail_unless((R2D*llh[0] >= -90) && (R2D*llh[0] <= 90),
      "Converting random WGS86 ECEF gives latitude out of bounds.\n"
      "Initial ECEF: %f, %f, %f\n"
      "LLH: %f, %f, %f\n",
      ecef_init[0], ecef_init[1], ecef_init[2],
      R2D*llh[0], R2D*llh[1], llh[2]
  );

  fail_unless((R2D*llh[1] >= -180) && (R2D*llh[1] <= 180),
      "Converting random WGS86 ECEF gives longitude out of bounds.\n"
      "Initial ECEF: %f, %f, %f\n"
      "LLH: %f, %f, %f\n",
      ecef_init[0], ecef_init[1], ecef_init[2],
      R2D*llh[0], R2D*llh[1], llh[2]
  );

  fail_unless(llh[2] > -EARTH_A,
      "Converting random WGS86 ECEF gives height out of bounds.\n"
      "Initial ECEF: %f, %f, %f\n"
      "LLH: %f, %f, %f\n",
      ecef_init[0], ecef_init[1], ecef_init[2],
      R2D*llh[0], R2D*llh[1], llh[2]
  );
}
END_TEST

/* Check simply that passing the ECEF position the same as the
 * reference position returns (0, 0, 0) in NED frame */
START_TEST(test_random_wgsecef2ned_d_0) {
  s32 i, j;
  double ned[3];

  seed_rng();
  for (i = 0; i < 222; i++) {
    const double ecef[3] = {frand(-1e8, 1e8),
                            frand(-1e8, 1e8),
                            frand(-1e8, 1e8)};
    wgsecef2ned_d(ecef, ecef, ned);
    for (j = 0; j < 3; j++)
      fail_unless(fabs(ned[j]) < 1e-8,
                  "NED vector to reference ECEF point "
                  "has nonzero element %d: %lf\n"
                  "(point was <%lf %lf %lf>)\n",
                  ned[j], ecef[0], ecef[1], ecef[2]);
  }
}
END_TEST

Suite* coord_system_suite(void)
{
  Suite *s = suite_create("Coordinate systems");

  /* Core test case */
  TCase *tc_core = tcase_create("Core");
  tcase_add_loop_test(tc_core, test_wgsllh2ecef, 0, NUM_COORDS);
  tcase_add_loop_test(tc_core, test_wgsecef2llh, 0, NUM_COORDS);
  tcase_add_loop_test(tc_core, test_wgsllh2ecef2llh, 0, NUM_COORDS);
  tcase_add_loop_test(tc_core, test_wgsecef2llh2ecef, 0, NUM_COORDS);
  suite_add_tcase(s, tc_core);

  TCase *tc_random = tcase_create("Random");
  tcase_add_loop_test(tc_random, test_random_wgsllh2ecef2llh, 0, 100);
  tcase_add_loop_test(tc_random, test_random_wgsecef2llh2ecef, 0, 100);
  tcase_add_loop_test(tc_random, test_random_wgsecef2ned_d_0, 0, 100);
  suite_add_tcase(s, tc_random);

  return s;
}


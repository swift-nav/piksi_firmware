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

#define NUM_COORDS 4
const double const llhs[NUM_COORDS][3] = {
  {0, 0, 0},       /* On the Equator and Prime Meridian. */
  {0, 180*D2R, 0}, /* On the Equator. */
  {90*D2R, 0, 0},  /* North pole. */
  {-90*D2R, 0, 0}, /* South pole. */
};
const double const xyzs[NUM_COORDS][3] = {
  {EARTH_A, 0, 0},
  {-EARTH_A, 0, 0},
  {0, 0, EARTH_B},
  {0, 0, -EARTH_B},
};

START_TEST(test_wgsllh2xyz)
{
  double xyz[3];

  wgsllh2xyz(llhs[_i], xyz);

  for (int n=0; n<3; n++) {
    fail_unless(!isnan(xyz[n]), "NaN in output from wgsllh2xyz.");
    double err = fabs(xyz[n] - xyzs[_i][n]);
    fail_unless(err < MAX_DIST_ERROR_M,
      "Conversion from WGS84 LLH to XYZ has >1e-6m error:\n"
      "LLH: %f, %f, %f\n"
      "X error (mm): %g\nY error (mm): %g\nZ error (mm): %g",
      llhs[_i][0]*R2D, llhs[_i][1]*R2D, llhs[_i][2],
      (xyz[0] - xyzs[_i][0])*1e3,
      (xyz[1] - xyzs[_i][1])*1e3,
      (xyz[2] - xyzs[_i][2])*1e3
    );
  }
}
END_TEST

START_TEST(test_wgsxyz2llh)
{
  double llh[3];

  wgsxyz2llh(xyzs[_i], llh);

  for (int n=0; n<3; n++) {
    fail_unless(!isnan(llh[n]), "NaN in output from wgsxyz2llh.");
  }

  double lat_err = fabs(llh[0] - llhs[_i][0]);
  double lon_err = fabs(llh[1] - llhs[_i][1]);
  double hgt_err = fabs(llh[2] - llhs[_i][2]);
  fail_unless((lat_err < MAX_ANGLE_ERROR_RAD) ||
              (lon_err < MAX_ANGLE_ERROR_RAD) ||
              (hgt_err < MAX_DIST_ERROR_M),
    "Conversion from WGS84 XYZ to LLH has >1e-6 {rad, m} error:\n"
    "XYZ: %f, %f, %f\n"
    "Lat error (arc sec): %g\nLon error (arc sec): %g\nH error (mm): %g",
    xyzs[_i][0], xyzs[_i][1], xyzs[_i][2],
    (llh[0] - llhs[_i][0])*(R2D*3600),
    (llh[1] - llhs[_i][1])*(R2D*3600),
    (llh[2] - llhs[_i][2])*1e3
  );
}
END_TEST

START_TEST(test_wgsllh2xyz2llh)
{
  double xyz[3];
  double llh[3];

  wgsllh2xyz(llhs[_i], xyz);
  wgsxyz2llh(xyz, llh);

  for (int n=0; n<3; n++) {
    fail_unless(!isnan(llh[n]), "NaN in LLH output from conversion.");
    fail_unless(!isnan(xyz[n]), "NaN in XYZ output from conversion.");
  }

  double lat_err = fabs(llh[0] - llhs[_i][0]);
  double lon_err = fabs(llh[1] - llhs[_i][1]);
  double hgt_err = fabs(llh[2] - llhs[_i][2]);
  fail_unless((lat_err < MAX_ANGLE_ERROR_RAD) ||
              (lon_err < MAX_ANGLE_ERROR_RAD) ||
              (hgt_err < MAX_DIST_ERROR_M),
    "Converting WGS84 LLH to XYZ and back again does not return the "
    "original values.\n"
    "Initial LLH: %f, %f, %f\n"
    "XYZ: %f, %f, %f\n"
    "Final LLH: %f, %f, %f\n"
    "Lat error (arc sec): %g\nLon error (arc sec): %g\nH error (mm): %g",
    R2D*llhs[_i][0], R2D*llhs[_i][1], llhs[_i][2],
    xyz[0], xyz[1], xyz[2],
    R2D*llh[0], R2D*llh[1], llh[2],
    (llh[0] - llhs[_i][0])*(R2D*3600),
    (llh[1] - llhs[_i][1])*(R2D*3600),
    (llh[2] - llhs[_i][2])*1e3
  );
}
END_TEST

START_TEST(test_wgsxyz2llh2xyz)
{
  double llh[3];
  double xyz[3];

  wgsxyz2llh(xyzs[_i], llh);
  wgsllh2xyz(llh, xyz);

  for (int n=0; n<3; n++) {
    fail_unless(!isnan(llh[n]), "NaN in LLH output from conversion.");
    fail_unless(!isnan(xyz[n]), "NaN in XYZ output from conversion.");
  }

  for (int n=0; n<3; n++) {
    double err = fabs(xyz[n] - xyzs[_i][n]);
    fail_unless(err < MAX_DIST_ERROR_M,
      "Converting WGS84 XYZ to LLH and back again does not return the "
      "original values.\n"
      "Initial XYZ: %f, %f, %f\n"
      "LLH: %f, %f, %f\n"
      "Final XYZ: %f, %f, %f\n"
      "X error (mm): %g\nY error (mm): %g\nZ error (mm): %g",
      xyzs[_i][0], xyzs[_i][1], xyzs[_i][2],
      R2D*llh[0], R2D*llh[1], llh[2],
      xyz[0], xyz[1], xyz[2],
      (xyz[0] - xyzs[_i][0])*1e3,
      (xyz[1] - xyzs[_i][1])*1e3,
      (xyz[2] - xyzs[_i][2])*1e3
    );
  }
}
END_TEST

START_TEST(test_random_wgsllh2xyz2llh)
{
  double xyz[3];
  double llh_init[3];
  double llh[3];

  seed_rng();

  llh_init[0] = D2R*frand(-90, 90);
  llh_init[1] = D2R*frand(-180, 180);
  llh_init[2] = frand(-EARTH_A, 4*EARTH_A);

  wgsllh2xyz(llh_init, xyz);
  wgsxyz2llh(xyz, llh);

  for (int n=0; n<3; n++) {
    fail_unless(!isnan(llh[n]), "NaN in LLH output from conversion.");
    fail_unless(!isnan(xyz[n]), "NaN in XYZ output from conversion.");
  }

  double lat_err = fabs(llh[0] - llh_init[0]);
  double lon_err = fabs(llh[1] - llh_init[1]);
  double hgt_err = fabs(llh[2] - llh_init[2]);
  fail_unless((lat_err < MAX_ANGLE_ERROR_RAD) ||
              (lon_err < MAX_ANGLE_ERROR_RAD) ||
              (hgt_err < MAX_DIST_ERROR_M),
    "Converting random WGS84 LLH to XYZ and back again does not return the "
    "original values.\n"
    "Initial LLH: %f, %f, %f\n"
    "XYZ: %f, %f, %f\n"
    "Final LLH: %f, %f, %f\n"
    "Lat error (arc sec): %g\nLon error (arc sec): %g\nH error (mm): %g",
    R2D*llh_init[0], R2D*llh_init[1], llh_init[2],
    xyz[0], xyz[1], xyz[2],
    R2D*llh[0], R2D*llh[1], llh[2],
    (llh[0] - llh_init[0])*(R2D*3600),
    (llh[1] - llh_init[1])*(R2D*3600),
    (llh[2] - llh_init[2])*1e3
  );

  fail_unless((R2D*llh[0] >= -90) && (R2D*llh[0] <= 90),
      "Converting random WGS86 XYZ gives latitude out of bounds.\n"
      "XYZ: %f, %f, %f\n"
      "LLH: %f, %f, %f\n",
      xyz[0], xyz[1], xyz[2],
      R2D*llh[0], R2D*llh[1], llh[2]
  );

  fail_unless((R2D*llh[1] >= -180) && (R2D*llh[1] <= 180),
      "Converting random WGS86 XYZ gives longitude out of bounds.\n"
      "XYZ: %f, %f, %f\n"
      "LLH: %f, %f, %f\n",
      xyz[0], xyz[1], xyz[2],
      R2D*llh[0], R2D*llh[1], llh[2]
  );

  fail_unless(llh[2] > -EARTH_A,
      "Converting random WGS86 XYZ gives height out of bounds.\n"
      "XYZ: %f, %f, %f\n"
      "LLH: %f, %f, %f\n",
      xyz[0], xyz[1], xyz[2],
      R2D*llh[0], R2D*llh[1], llh[2]
  );
}
END_TEST

START_TEST(test_random_wgsxyz2llh2xyz)
{
  double xyz_init[3];
  double llh[3];
  double xyz[3];

  seed_rng();

  xyz_init[0] = frand(-4*EARTH_A, 4*EARTH_A);
  xyz_init[1] = frand(-4*EARTH_A, 4*EARTH_A);
  xyz_init[2] = frand(-4*EARTH_A, 4*EARTH_A);

  wgsxyz2llh(xyz_init, llh);
  wgsllh2xyz(llh, xyz);

  for (int n=0; n<3; n++) {
    fail_unless(!isnan(llh[n]), "NaN in LLH output from conversion.");
    fail_unless(!isnan(xyz[n]), "NaN in XYZ output from conversion.");
  }

  for (int n=0; n<3; n++) {
    double err = fabs(xyz[n] - xyz_init[n]);
    fail_unless(err < MAX_DIST_ERROR_M,
      "Converting random WGS84 XYZ to LLH and back again does not return the "
      "original values.\n"
      "Initial XYZ: %f, %f, %f\n"
      "LLH: %f, %f, %f\n"
      "Final XYZ: %f, %f, %f\n"
      "X error (mm): %g\nY error (mm): %g\nZ error (mm): %g",
      xyz_init[0], xyz_init[1], xyz_init[2],
      R2D*llh[0], R2D*llh[1], llh[2],
      xyz[0], xyz[1], xyz[2],
      (xyz[0] - xyz_init[0])*1e3,
      (xyz[1] - xyz_init[1])*1e3,
      (xyz[2] - xyz_init[2])*1e3
    );
  }

  fail_unless((R2D*llh[0] >= -90) && (R2D*llh[0] <= 90),
      "Converting random WGS86 XYZ gives latitude out of bounds.\n"
      "Initial XYZ: %f, %f, %f\n"
      "LLH: %f, %f, %f\n",
      xyz_init[0], xyz_init[1], xyz_init[2],
      R2D*llh[0], R2D*llh[1], llh[2]
  );

  fail_unless((R2D*llh[1] >= -180) && (R2D*llh[1] <= 180),
      "Converting random WGS86 XYZ gives longitude out of bounds.\n"
      "Initial XYZ: %f, %f, %f\n"
      "LLH: %f, %f, %f\n",
      xyz_init[0], xyz_init[1], xyz_init[2],
      R2D*llh[0], R2D*llh[1], llh[2]
  );

  fail_unless(llh[2] > -EARTH_A,
      "Converting random WGS86 XYZ gives height out of bounds.\n"
      "Initial XYZ: %f, %f, %f\n"
      "LLH: %f, %f, %f\n",
      xyz_init[0], xyz_init[1], xyz_init[2],
      R2D*llh[0], R2D*llh[1], llh[2]
  );
}
END_TEST

Suite* coord_system_suite(void)
{
  Suite *s = suite_create("Coordinate systems");

  /* Core test case */
  TCase *tc_core = tcase_create("Core");
  tcase_add_loop_test(tc_core, test_wgsllh2xyz, 0, NUM_COORDS);
  tcase_add_loop_test(tc_core, test_wgsxyz2llh, 0, NUM_COORDS);
  tcase_add_loop_test(tc_core, test_wgsllh2xyz2llh, 0, NUM_COORDS);
  tcase_add_loop_test(tc_core, test_wgsxyz2llh2xyz, 0, NUM_COORDS);
  suite_add_tcase(s, tc_core);

  TCase *tc_random = tcase_create("Random");
  tcase_add_loop_test(tc_random, test_random_wgsllh2xyz2llh, 0, 100);
  tcase_add_loop_test(tc_random, test_random_wgsxyz2llh2xyz, 0, 100);
  suite_add_tcase(s, tc_random);

  return s;
}


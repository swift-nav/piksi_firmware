#include <check.h>

#include "../coord_system.h"

START_TEST(test_wgsllh2xyz2llh)
{
  const double const llh[3] = {0, 0, 0};
  double xyz[3];
  double llh_out[3];

  wgsllh2xyz(llh, xyz);
  wgsxyz2llh(xyz, llh_out);

  for (int i=0; i<3; i++) {
    if (llh_out[i] != llh[0])
      fail("Converting WGS84 LLH to XYZ and back again does not return the original values.");
  }
}
END_TEST

Suite* coord_system_suite(void)
{
  Suite *s = suite_create("Coordinate systems");

  /* Core test case */
  TCase *tc_core = tcase_create("Core");
  tcase_add_test(tc_core, test_wgsllh2xyz2llh);
  suite_add_tcase(s, tc_core);

  return s;
}


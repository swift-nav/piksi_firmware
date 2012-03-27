#include <stdlib.h>
#include <check.h>

#include "check_suites.h"

int main(void)
{
  int number_failed;

  Suite *s = coord_system_suite();

  SRunner *sr = srunner_create(s);
  srunner_add_suite(sr, linear_algebra_suite());
  
  srunner_run_all(sr, CK_NORMAL);
  number_failed = srunner_ntests_failed(sr);
  srunner_free(sr);
  return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}

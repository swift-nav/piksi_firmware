
#include <math.h>

/* Simple Black model, taken from GPSTk SimpleTropModel class. */

double dry_zenith_delay(void)
{
  return 2.235486646978727;
}

double dry_mapping_function(double elevation)
{
  double d = cos(elevation);
  d /= 1.001012704615527;
  return (1.0 / sqrt(1.0 - d*d));
}

double wet_zenith_delay(void)
{
  return 0.122382715318184;
}

double wet_mapping_function(double elevation)
{
  double d = cos(elevation);
  d /= 1.000282213715744;
  return (1.0 / sqrt(1.0 - d*d));
}

double tropo_correction(double elevation)
{
  if (elevation < 0)
    return 0;

  return (dry_zenith_delay() * dry_mapping_function(elevation)
        + wet_zenith_delay() * wet_mapping_function(elevation));
}

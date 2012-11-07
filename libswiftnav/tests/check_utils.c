#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "check_utils.h"

void seed_rng(void) {
  FILE* fp = fopen("/dev/urandom", "r");
  unsigned int seed;
  int out;
  if ((out = fread(&seed, sizeof(seed), 1, fp)) == sizeof(seed))
    srandom(seed);
}

double frand(double fmin, double fmax) {
  double f = (double)random() / RAND_MAX;
  return fmin + f * (fmax - fmin);
}

u32 sizerand(u32 sizemax) {
  double f = (double)random() / RAND_MAX;
  return (u32) ceil(f * sizemax);
}

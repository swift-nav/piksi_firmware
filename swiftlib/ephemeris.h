#ifndef SWIFTNAV_EPHEMERIS_H
#define SWIFTNAV_EPHEMERIS_H

#include <stdint.h>

typedef struct {
  double tgd;
  double crs, crc, cuc, cus, cic, cis;
  double dn, m0, ecc, sqrta, omega0, omegadot, w, inc, inc_dot;
  double af0, af1, af2;
  uint32_t toe, toc;
  unsigned int valid:1;
  unsigned int healthy:1;


} ephemeris_t;

int calc_sat_pos(double pos[3], double vel[3], 
             double *clock_err, double *clock_rate_err,
             const ephemeris_t *ephemeris,
             double time_of_transmit);

#endif

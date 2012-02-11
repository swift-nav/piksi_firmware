#ifndef SWIFTNAV_EPHEMERIS_H
#define SWIFTNAV_EPHEMERIS_H

//typedef struct __attribute__((packed)) {
  /* 
   * Be careful of stuct packing to avoid (very mild) slowness,
   * try to keep all the types aligned i.e. put the 64bit 
   * things together at the top, then the 32bit ones etc.
   */
typedef struct {
  double tgd;
  double crs, crc, cuc, cus, cic, cis;
  double dn, m0, ecc, sqrta, omega0, omegadot, w, inc, inc_dot;
  double af0, af1, af2;
  uint32_t toe, toc;
  uint32_t valid_for_pvt;
  uint32_t tow, week_num, health;
  uint8_t sv;
  uint8_t nav_data_state;
  uint8_t next_subframe;
} ephemeris_t;

#endif

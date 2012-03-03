
#ifndef SWIFTLIB_TRACK_H
#define SWIFTLIB_TRACK_H

#include "common.h"
#include "ephemeris.h"

typedef struct {
  u8 prn;
  double code_phase_chips;
  double code_phase_rate;
  double carrier_phase;
  double carrier_freq;
  u32 time_of_week_ms;
  double receiver_time;
  double snr;
} channel_measurement_t;

typedef struct {
  double pseudorange;
  double pseudorange_rate;
  double TOT;
  double sat_pos[3];
  double sat_vel[3];
} navigation_measurement_t;

void calc_loop_coeff(double BW, double zeta, double k, double *tau1, double *tau2);

void calc_navigation_measurement(u8 n_channels, channel_measurement_t meas[], navigation_measurement_t nav_meas[], double nav_time, ephemeris_t ephemerides[]);

#endif

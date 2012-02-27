
#ifndef SWIFTLIB_TRACK_H
#define SWIFTLIB_TRACK_H

typedef struct {
  u8 prn;
  double code_phase_chips;
  double code_phase_rate_chips_per_second;
  double carrier_phase_radians;
  double carrier_freq_hz;
  u32 time_of_week_ms;
  u32 receiver_time_samples;
  double snr;
} channel_measurement_t;

typedef struct {
  double pseudorange_meters;
  double pseudorange_rate;
  double sat_pos[3];
  double sat_vel[3];
} navigation_measurement_t;

void calc_pseudoranges(u8 n_channels, channel_measurement_t meas[], navigation_measurement_t nav_meas[], u32 nav_time_samples);

#endif

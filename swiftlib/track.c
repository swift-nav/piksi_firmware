#include "track.h"
#include "ephemeris.h"

void calc_pseudoranges(u8 n_channels, channel_measurement_t meas[], navigation_measurement_t nav_meas[], u32 nav_time_samples)
{
  double TOTs[n_channels];
  double mean_TOT = 0;

  for (u8 i=0; i<n_channels; i++) {
    TOTs[i] = 1e-3*meas[i].time_of_week_ms;
    TOTs[i] += meas[i].code_phase_chips / 1.023e6;
    TOTs[i] += (nav_time_samples - meas[i].receiver_time_samples) * meas[i].code_phase_rate_chips_per_second / 1.023e6;

    mean_TOT += TOTs[i];
    nav_meas[i].pseudorange_rate = NAV_C * -meas[i].carrier_freq_hz / L1_HZ;
  }

  mean_TOT = mean_TOT/n_channels;

  double clock_err, clock_rate_err;

  for (u8 i=0; i<n_channels; i++) {
    calc_sat_pos(nav_meas[i].sat_pos, nav_meas[i].sat_vel, &clock_err, &clock_rate_err, prn, TOTs[i]);
    nav_meas[i].pseudorange = (mean_TOT - TOTs[i])*NAV_C + NOMINAL_RANGE;
    nav_meas[i].pseudorange += clock_err*NAV_C;
    nav_meas[i].pseudorange_rate -= clock_rate_err*NAV_C;
  }
}


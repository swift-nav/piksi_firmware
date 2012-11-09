from common cimport *
from ephemeris_c cimport ephemeris_t

cdef extern from "track.h":
  ctypedef struct channel_measurement_t:
    u8 prn
    double code_phase_chips
    double code_phase_rate
    double carrier_phase
    double carrier_freq
    u32 time_of_week_ms
    double receiver_time
    double snr

  ctypedef struct navigation_measurement_t:
    double pseudorange
    double pseudorange_rate
    double TOT
    double sat_pos[3]
    double sat_vel[3]

  void track_correlate(s8* samples, s8* code, double* init_code_phase, double code_step, double* init_carr_phase, double carr_step, double* I_E, double* Q_E, double* I_P, double* Q_P, double* I_L, double* Q_L, u32* num_samples)

  void calc_navigation_measurement_(u8 n_channels, channel_measurement_t* meas[], navigation_measurement_t* nav_meas[], double nav_time, ephemeris_t* ephemerides[])


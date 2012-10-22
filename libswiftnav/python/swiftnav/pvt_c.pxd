from common cimport *
from track_c cimport navigation_measurement_t

cdef extern from "pvt.h":
  ctypedef struct dops_t:
    double pdop
    double gdop
    double tdop
    double hdop
    double vdop

  ctypedef struct gnss_solution:
    double pos_llh[3]
    double pos_ecef[3]
    double pos_ned[3]
    double vel_ecef[3]
    double vel_ned[3]
    double err_cov[7]
    double time
    u8 gps_solution_valid
    u8 n_used

  u8 calc_PVT(u8 n_used,
              navigation_measurement_t nav_meas[],
              gnss_solution *soln,
              dops_t *dops)


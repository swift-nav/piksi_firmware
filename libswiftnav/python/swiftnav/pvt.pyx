cimport pvt_c
from libc.stdlib cimport malloc, free

from common cimport *
from track_c cimport navigation_measurement_t
from track cimport NavigationMeasurement

cdef class Solution:
  def __repr__(self):
    return "<Solution %f, %f, %f>" % (57.2957795*self.pos_llh[0],
                                      57.2957795*self.pos_llh[1],
                                      self.pos_llh[2])

  property pos_llh:
    def __get__(self):
      return (self.soln.pos_llh[0],
              self.soln.pos_llh[1],
              self.soln.pos_llh[2])

  property pos_ecef:
    def __get__(self):
      return (self.soln.pos_ecef[0],
              self.soln.pos_ecef[1],
              self.soln.pos_ecef[2])

  property vel_ned:
    def __get__(self):
      return (self.soln.vel_ned[0],
              self.soln.vel_ned[1],
              self.soln.vel_ned[2])

  property vel_ecef:
    def __get__(self):
      return (self.soln.vel_ecef[0],
              self.soln.vel_ecef[1],
              self.soln.vel_ecef[2])

  property tow:
    def __get__(self):
      return self.soln.time

def calc_PVT(nav_meas):
  n_used = len(nav_meas)
  cdef navigation_measurement_t* nav_meas_array = <navigation_measurement_t*>malloc(n_used*sizeof(navigation_measurement_t))

  for n in range(n_used):
    nav_meas_array[n] = (<NavigationMeasurement?>nav_meas[n]).meas

  s = Solution()
  pvt_c.calc_PVT(n_used, nav_meas_array, &(s.soln), &(s.dops))

  free(nav_meas_array)

  return s

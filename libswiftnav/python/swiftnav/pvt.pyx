cimport pvt_c
from libc.stdlib cimport malloc, free
from libc.string cimport memcpy

from common cimport *
from track_c cimport navigation_measurement_t
from track cimport NavigationMeasurement

def foo(nav_meas):
  n_used = len(nav_meas)
  cdef navigation_measurement_t* nav_meas_array = <navigation_measurement_t*>malloc(n_used*sizeof(navigation_measurement_t))
  cdef pvt_c.dops_t dops
  cdef pvt_c.gnss_solution soln

  for n in range(n_used):
    #nav_meas_array[n] = (<NavigationMeasurement?>nav_meas[n]).meas
    memcpy(&nav_meas_array[n],
           &((<NavigationMeasurement?>nav_meas[n]).meas),
           sizeof(navigation_measurement_t))

  pvt_c.calc_PVT(n_used, nav_meas_array, &soln, &dops)

  free(nav_meas_array)

  #print soln.pos_ecef[0], soln.pos_ecef[1], soln.pos_ecef[2]
  return (soln.pos_llh[1], soln.pos_llh[0], soln.pos_llh[2])

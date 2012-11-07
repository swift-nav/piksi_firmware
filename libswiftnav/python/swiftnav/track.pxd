from track_c cimport navigation_measurement_t

cdef class NavigationMeasurement:
  cdef navigation_measurement_t meas

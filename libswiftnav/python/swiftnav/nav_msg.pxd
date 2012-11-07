cimport nav_msg_c
cimport ephemeris_c
from cpython cimport bool

cdef class NavMsg:
  cdef nav_msg_c.nav_msg_t state
  cdef ephemeris_c.ephemeris_t eph
  cdef readonly bool eph_valid

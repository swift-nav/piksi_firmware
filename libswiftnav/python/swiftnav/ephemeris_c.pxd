from common cimport *

cdef extern from "ephemeris.h":
  ctypedef struct ephemeris_t:
    u8 valid
    u16 wn



from libc.stdint cimport int8_t, int16_t, int32_t, uint8_t, uint16_t, uint32_t

cdef extern from "common.h":
  ctypedef int8_t s8
  ctypedef int16_t s16
  ctypedef int32_t s32
  ctypedef uint8_t u8
  ctypedef uint16_t u16
  ctypedef uint32_t u32


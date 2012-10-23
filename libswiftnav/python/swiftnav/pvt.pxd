cimport pvt_c

cdef class Solution:
  cdef pvt_c.gnss_solution soln
  cdef pvt_c.dops_t dops


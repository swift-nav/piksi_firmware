from common cimport *
from ephemeris_c cimport ephemeris_t
from libcpp cimport bool

cdef extern from "nav_msg.h":
  ctypedef struct nav_msg_t:
    pass

  void nav_msg_init(nav_msg_t *n)
  u32 nav_msg_update(nav_msg_t *n, s32 corr_prompt_real)
  bool subframe_ready(nav_msg_t *n)
  void process_subframe(nav_msg_t *n, ephemeris_t *e)


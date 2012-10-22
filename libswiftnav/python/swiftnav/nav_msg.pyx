cdef class NavMsg:
  def __cinit__(self):
    nav_msg_c.nav_msg_init(&self.state)
    self.eph_valid = False

  def update(self, corr_prompt_real):
    tow = nav_msg_c.nav_msg_update(&self.state, corr_prompt_real)
    if nav_msg_c.subframe_ready(&self.state):
      #self.eph.valid = 0
      nav_msg_c.process_subframe(&self.state, &self.eph)
      if self.eph.valid:
        self.eph_valid = True
      else:
        self.eph_valid = False
        #return Ephemeris(&self.eph)
    return tow if tow != 0 else None

  #property cptr:
    #def __get__(self):
      #return &self.eph



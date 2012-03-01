from enthought.traits.api import Str, Instance, Dict, HasTraits, Array, Float, on_trait_change, List, Int
from enthought.traits.ui.api import Item, View, HGroup

import struct

MSG_SOLUTION = 0x50
MSG_SOLUTION_DOPS = 0x51

class SolutionView(HasTraits):
  python_console_cmds = Dict()
  pdop = Float()
  gdop = Float()
  tdop = Float()
  hdop = Float()
  vdop = Float()
  pos_llh = Array(dtype=float, shape=(3,))
  n_used = Int()

  traits_view = View(
    HGroup(
      Item('pdop', label='PDOP', format_str='%.1f'),
      Item('gdop', label='GDOP', format_str='%.1f'),
      Item('tdop', label='TDOP', format_str='%.1f'),
      Item('hdop', label='HDOP', format_str='%.1f'),
      Item('vdop', label='VDOP', format_str='%.1f'),
    ),
    Item('n_used'),
    Item('pos_llh'),
  )

  def solution_callback(self, data):
    soln = struct.unpack('<3d3d3d3d7ddBB', data)
    self.pos_llh = soln[0:3]
    pos_xyz = soln[3:6]
    vel_xyz = soln[6:9]
    vel_ned = soln[9:12]
    err_cov = soln[12:19]
    time = soln[19]
    soln_valid = soln[20]
    self.n_used = soln[21]

  def dops_callback(self, data):
    self.pdop, self.dgop, self.tdop, self.hdop, self.vdop = struct.unpack('<ddddd', data)

  def __init__(self, link):
    super(SolutionView, self).__init__()

    self.link = link
    self.link.add_callback(MSG_SOLUTION, self.solution_callback)
    self.link.add_callback(MSG_SOLUTION_DOPS, self.dops_callback)

    self.python_console_cmds = {
      'solution': self
    }


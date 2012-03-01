from enthought.traits.api import Str, Instance, Dict, HasTraits, Array, Float, on_trait_change, List, Int, Button
from enthought.traits.ui.api import Item, View, HGroup

from enthought.chaco.api import BarPlot, ArrayDataSource, DataRange1D, LinearMapper, OverlayPlotContainer, LabelAxis, PlotAxis, ArrayPlotData, Plot
from enthought.enable.api import ComponentEditor, Component

import struct
import math

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
  pos_ned = Array(dtype=float, shape=(3,))
  ns = List()
  es = List()
  ds = List()
  n_used = Int()
  dist = Float()

  plot = Instance(Plot)
  plot_data = Instance(ArrayPlotData)
  clear = Button()

  traits_view = View(
    HGroup(
      Item('pdop', label='PDOP', format_str='%.1f'),
      Item('gdop', label='GDOP', format_str='%.1f'),
      Item('tdop', label='TDOP', format_str='%.1f'),
      Item('hdop', label='HDOP', format_str='%.1f'),
      Item('vdop', label='VDOP', format_str='%.1f'),
    ),
    Item('clear'),
    HGroup(
      Item('pos_llh'),
      Item('pos_ned'),
      Item('dist'),
    ),
    Item(
      'plot',
      editor = ComponentEditor(bgcolor = (0.8,0.8,0.8)),
    )
  )

  def _clear_fired(self):
    self.ns = []
    self.es = []
    self.ds = []

  def solution_callback(self, data):
    soln = struct.unpack('<3d3d3d3d3d7ddBB', data)
    self.pos_llh = [soln[0]*(180/math.pi), soln[1]*(180/math.pi), soln[2]]
    pos_xyz = soln[3:6]
    self.pos_ned = soln[6:9]
    vel_xyz = soln[9:12]
    vel_ned = soln[12:15]
    err_cov = soln[15:21]
    time = soln[21]
    soln_valid = soln[22]
    self.n_used = soln[23]

    self.dist = math.sqrt(self.pos_ned[0]**2 + self.pos_ned[1]**2 + self.pos_ned[2]**2)

    if self.dist < 3000:
      self.ns.append(self.pos_ned[0])
      self.es.append(self.pos_ned[1])
      self.ds.append(self.pos_ned[2])
    else:
      print "Whacky solution detected!"

    self.plot_data.set_data('n', self.ns)
    self.plot_data.set_data('e', self.es)
    self.plot_data.set_data('h', self.ds)
    t = range(len(self.ds))
    self.plot_data.set_data('t', t)

  def dops_callback(self, data):
    self.pdop, self.dgop, self.tdop, self.hdop, self.vdop = struct.unpack('<ddddd', data)

  def __init__(self, link):
    super(SolutionView, self).__init__()

    self.link = link
    self.link.add_callback(MSG_SOLUTION, self.solution_callback)
    self.link.add_callback(MSG_SOLUTION_DOPS, self.dops_callback)

    self.plot_data = ArrayPlotData(n=[0.0], e=[0.0], h=[0.0], t=[0.0])
    self.plot = Plot(self.plot_data) #, auto_colors=colours_list)
    self.plot.plot(('n', 'e'), type='line', color='blue')

    self.python_console_cmds = {
      'solution': self
    }


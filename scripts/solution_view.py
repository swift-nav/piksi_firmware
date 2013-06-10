from traits.api import Str, Instance, Dict, HasTraits, Array, Float, on_trait_change, List, Int, Button
from traitsui.api import Item, View, HGroup, VGroup, ArrayEditor

from chaco.api import BarPlot, ArrayDataSource, DataRange1D, LinearMapper, OverlayPlotContainer, LabelAxis, PlotAxis, ArrayPlotData, Plot
from enable.api import ComponentEditor, Component

import struct
import math
import numpy as np

TRACK_N_CHANNELS = 10

MSG_SOLUTION = 0x50
MSG_SOLUTION_DOPS = 0x51

colours_list = ['red', 'blue']

class SolutionView(HasTraits):
  python_console_cmds = Dict()
  pdop = Float()
  gdop = Float()
  tdop = Float()
  hdop = Float()
  vdop = Float()
  pos_llh = Array(dtype=float, shape=(1,3,))
  lats = List()
  lngs = List()
  alts = List()
  n_used = Int()

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
    Item('clear', show_label = False),
    HGroup(
      Item('pos_llh', editor = ArrayEditor(width = 80)),
      Item('n_used')
    ),
    Item(
      'plot',
      show_label = False,
      editor = ComponentEditor(bgcolor = (0.8,0.8,0.8)),
    )
  )

  def _clear_fired(self):
    self.lats = []
    self.lngs = []
    self.alts = []

  def solution_callback(self, data):
    soln = struct.unpack('<3d3d3d3d7ddHBB', data)
    self.pos_llh = [[soln[0]*(180/math.pi), soln[1]*(180/math.pi), soln[2]]]
    pos_xyz = soln[3:6]
    vel_ned = soln[6:9]
    vel_xyz = soln[9:12]
    err_cov = soln[12:19]
    tow = soln[19]
    week_num = soln[20]
    soln_valid = soln[21]

    self.lats.append(self.pos_llh[0][0])
    self.lngs.append(self.pos_llh[0][1])
    self.alts.append(self.pos_llh[0][2])

    self.plot_data.set_data('lat', self.lats)
    self.plot_data.set_data('lng', self.lngs)
    self.plot_data.set_data('alt', self.alts)
    t = range(len(self.lats))
    self.plot_data.set_data('t', t)

    self.n_used = soln[22]

  def dops_callback(self, data):
    self.pdop, self.dgop, self.tdop, self.hdop, self.vdop = struct.unpack('<ddddd', data)

  def __init__(self, link):
    super(SolutionView, self).__init__()

    self.link = link
    self.link.add_callback(MSG_SOLUTION, self.solution_callback)
    self.link.add_callback(MSG_SOLUTION_DOPS, self.dops_callback)

    self.plot_data = ArrayPlotData(lat=[0.0], lng=[0.0], alt=[0.0], t=[0.0])
    self.plot = Plot(self.plot_data) #, auto_colors=colours_list)
    self.plot.plot(('lat', 'lng'), type='scatter', color='blue', marker='plus')

    #self.plot.plot(('ref_n', 'ref_e'),
        #type='scatter',
        #color='red',
        #marker='cross',
        #marker_size=10,
        #line_width=1.5
    #)

    self.python_console_cmds = {
      'solution': self
    }


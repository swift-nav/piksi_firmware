from traits.api import Str, Instance, Dict, HasTraits, Array, Float, on_trait_change, List, Int, Button
from traitsui.api import Item, View, HGroup, VGroup, ArrayEditor
from pyface.api import GUI

from chaco.api import BarPlot, ArrayDataSource, DataRange1D, LinearMapper, OverlayPlotContainer, LabelAxis, PlotAxis, ArrayPlotData, Plot
from enable.api import ComponentEditor, Component

import struct
import math
import numpy as np
from scipy.stats import nanmean
import datetime

MSG_SOLUTION = 0x50
MSG_SOLUTION_DOPS = 0x51

MEAN_LEN = 1024

colours_list = ['red', 'blue']

class SolutionView(HasTraits):
  python_console_cmds = Dict()
  pdop = Float()
  gdop = Float()
  tdop = Float()
  hdop = Float()
  vdop = Float()
  pos_llh = Array(dtype=float, shape=(1,3,))
  mean_llh = Array(dtype=float, shape=(1,3,))
  std_dev = Float()
  lats = List()
  lngs = List()
  alts = List()
  n_used = Int()
  gps_week = Int()
  gps_tow = Float()
  time = Str()

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
    HGroup(
      Item('time'),
      Item('gps_week', label = 'GPS Week:'),
      Item('gps_tow', label = 'GPS TOW:'),
    ),
    HGroup(
      Item('pos_llh', editor = ArrayEditor(width = 80), label = 'Lat/Lng/Alt:'),
      Item('n_used', label='Sats used:')
    ),
    HGroup(
      Item('mean_llh', editor = ArrayEditor(width = 80), label = 'Mean Lat/Lng/Alt:'),
      Item('std_dev', label='RMS dist:')
    ),
    Item('clear', show_label = False),
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
    self.llhs[:] = np.nan
    self.ecefs[:] = np.nan

  def _solution_callback(self, data):
    # Updating an ArrayPlotData isn't thread safe (see chaco issue #9), so
    # actually perform the update in the UI thread.
    GUI.invoke_later(self.solution_callback, data)

  def solution_callback(self, data):
    soln = struct.unpack('<3d3d3d3d7ddddHBB', data)
    self.pos_llh = [[soln[0]*(180/math.pi), soln[1]*(180/math.pi), soln[2]]]
    pos_ecef = soln[3:6]
    vel_ned = soln[6:9]
    vel_ecef = soln[9:12]
    err_cov = soln[12:19]
    clock_offset = soln[19]
    clock_bias = soln[20]
    self.gps_tow = soln[21]
    self.gps_week = soln[22]
    soln_valid = soln[23]
    self.n_used = soln[24]

    self.lats.append(self.pos_llh[0][0])
    self.lngs.append(self.pos_llh[0][1])
    self.alts.append(self.pos_llh[0][2])

    self.llhs[self.mean_n] = self.pos_llh
    self.ecefs[self.mean_n] = pos_ecef
    self.mean_n = (self.mean_n + 1) % MEAN_LEN
    mean_ecef = nanmean(self.ecefs)
    self.std_dev = np.sqrt(np.sum(nanmean((self.ecefs-mean_ecef)**2)))
    self.mean_llh = [nanmean(self.llhs)]

    lat_std_dev = np.sqrt(nanmean(self.llhs[:,0]**2) - self.mean_llh[0][0]**2)
    lng_std_dev = np.sqrt(nanmean(self.llhs[:,1]**2) - self.mean_llh[0][1]**2)

    p = np.linspace(0, 2*np.pi, 100)
    self.plot_data.set_data('region_lat', lat_std_dev*np.sin(p) + self.mean_llh[0][0])
    self.plot_data.set_data('region_lng', lng_std_dev*np.cos(p) + self.mean_llh[0][1])

    self.plot_data.set_data('ref_lat', [self.mean_llh[0][0]])
    self.plot_data.set_data('ref_lng', [self.mean_llh[0][1]])

    self.plot_data.set_data('lat', self.lats)
    self.plot_data.set_data('lng', self.lngs)
    self.plot_data.set_data('alt', self.alts)
    t = range(len(self.lats))
    self.plot_data.set_data('t', t)

    t = datetime.datetime(1980, 1, 5) + \
        datetime.timedelta(weeks=self.gps_week) + \
        datetime.timedelta(seconds=self.gps_tow)
    self.time = str(t)

  def dops_callback(self, data):
    self.pdop, self.dgop, self.tdop, self.hdop, self.vdop = struct.unpack('<ddddd', data)

  def __init__(self, link):
    super(SolutionView, self).__init__()

    self.plot_data = ArrayPlotData(lat=[0.0], lng=[0.0], alt=[0.0], t=[0.0], ref_lat=[0.0], ref_lng=[0.0], region_lat=[0.0], region_lng=[0.0])
    self.plot = Plot(self.plot_data) #, auto_colors=colours_list)

    self.plot.plot(('region_lat', 'region_lng'),
      type='polygon',
      face_color=(0,1,0,0.2),
      edge_color=(0,1,0,1)
    )

    self.plot.plot(('lat', 'lng'), type='scatter', color='blue', marker='plus')

    self.ecefs = np.empty((MEAN_LEN, 3)) * np.nan
    self.llhs = np.empty((MEAN_LEN, 3)) * np.nan
    self.mean_n = 0

    self.plot.plot(('ref_lat', 'ref_lng'),
        type='scatter',
        color='red',
        marker='cross',
        marker_size=10,
        line_width=1.5
    )

    self.link = link
    self.link.add_callback(MSG_SOLUTION, self._solution_callback)
    self.link.add_callback(MSG_SOLUTION_DOPS, self.dops_callback)

    self.python_console_cmds = {
      'solution': self
    }


from enthought.traits.api import Str, Instance, Dict, HasTraits, Array, Float, on_trait_change, List, Int, Button
from enthought.traits.ui.api import Item, View, HGroup, VGroup

from enthought.chaco.api import BarPlot, ArrayDataSource, DataRange1D, LinearMapper, OverlayPlotContainer, LabelAxis, PlotAxis, ArrayPlotData, Plot
from enthought.enable.api import ComponentEditor, Component

import struct
import math
import numpy as np

TRACK_N_CHANNELS = 5

MSG_SOLUTION = 0x50
MSG_SOLUTION_DOPS = 0x51
MSG_SOLUTION_PRS = 0x52
MSG_SOLUTION_PRED_PRS = 0x53

colours_list = ['red', 'blue']

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
  mean_ns = Float()
  es = List()
  mean_es = Float()
  ds = List()
  mean_ds = Float()
  mean_len = Int(50)
  mean_dist = Float()
  n_used = Int()
  dist = Float()

  plot = Instance(Plot)
  plot_data = Instance(ArrayPlotData)
  clear = Button()

  pr_plot = Instance(Plot)
  pr_plot_data = Instance(ArrayPlotData)
  prs = List()
  pred_prs = List()

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
      VGroup(
        Item('dist'),
        Item('mean_len'),
        Item('mean_dist'),
      ),
      VGroup(
        Item('mean_ns'),
        Item('mean_es'),
        Item('mean_ds'),
      ),
      Item('n_used')
    ),
    Item(
      'plot',
      editor = ComponentEditor(bgcolor = (0.8,0.8,0.8)),
    )
  )

  prs_view = View(
    Item(
      'pr_plot',
      editor = ComponentEditor(bgcolor = (0.8,0.8,0.8)),
      show_label = False,
    )
  )


  def _clear_fired(self):
    self.ns = []
    self.es = []
    self.ds = []

  def prs_callback(self, data):
    fmt = '<' + str(TRACK_N_CHANNELS) + 'd'
    self.prs.append(struct.unpack(fmt, data))
    prs = np.transpose(self.prs[-500:])
    t = range(len(prs[0]))
    self.pr_plot_data.set_data('t', t)
    for n in range(TRACK_N_CHANNELS):
      self.pr_plot_data.set_data('prs'+str(n), prs[n])

  def pred_prs_callback(self, data):
    return
    fmt = '<' + str(TRACK_N_CHANNELS) + 'd'
    self.pred_prs.append(struct.unpack(fmt, data))
    pred_prs = np.array(np.transpose(self.pred_prs[-500:]))
    prs = np.array(np.transpose(self.prs[-500:]))
    t = range(len(pred_prs[0]))
    self.pr_plot_data.set_data('t', t)
    for n in range(TRACK_N_CHANNELS):
      err = prs[n]-pred_prs[n]
      #err = err - (sum(err)/len(err))
      self.pr_plot_data.set_data('pred_prs'+str(n), err)

  def solution_callback(self, data):
    soln = struct.unpack('<3d3d3d3d3d7ddBB', data)
    self.pos_llh = [soln[0]*(180/math.pi), soln[1]*(180/math.pi), soln[2]]
    pos_xyz = soln[3:6]
    self.pos_ned = soln[6:9]
    vel_xyz = soln[9:12]
    vel_ned = soln[12:15]
    err_cov = soln[15:22]
    time = soln[22]
    soln_valid = soln[23]
    self.n_used = soln[24]

    self.dist = math.sqrt(self.pos_ned[0]**2 + self.pos_ned[1]**2 + self.pos_ned[2]**2)

    if self.dist < 3000:
      self.ns.append(self.pos_ned[0])
      self.es.append(self.pos_ned[1])
      self.ds.append(self.pos_ned[2])
    else:
      print "Whacky solution detected!"
    
    self.mean_ns = sum(self.ns[:self.mean_len])/len(self.ns[:self.mean_len])
    self.mean_es = sum(self.es[:self.mean_len])/len(self.es[:self.mean_len])
    self.mean_ds = sum(self.ds[:self.mean_len])/len(self.ds[:self.mean_len])
    self.mean_dist = math.sqrt(self.mean_ns**2 + self.mean_es**2 + self.mean_ds**2)

    self.plot_data.set_data('n', self.ns)
    self.plot_data.set_data('e', self.es)
    self.plot_data.set_data('h', self.ds)
    self.plot_data.set_data('ref_n', [0.0, self.mean_ns])
    self.plot_data.set_data('ref_e', [0.0, self.mean_es])
    t = range(len(self.ds))
    self.plot_data.set_data('t', t)

  def dops_callback(self, data):
    self.pdop, self.dgop, self.tdop, self.hdop, self.vdop = struct.unpack('<ddddd', data)

  def __init__(self, link):
    super(SolutionView, self).__init__()

    self.link = link
    self.link.add_callback(MSG_SOLUTION, self.solution_callback)
    self.link.add_callback(MSG_SOLUTION_DOPS, self.dops_callback)
    self.link.add_callback(MSG_SOLUTION_PRS, self.prs_callback)
    self.link.add_callback(MSG_SOLUTION_PRED_PRS, self.pred_prs_callback)

    self.plot_data = ArrayPlotData(n=[0.0], e=[0.0], h=[0.0], t=[0.0], ref_n=[0.0], ref_e=[0.0])
    self.plot = Plot(self.plot_data) #, auto_colors=colours_list)
    self.plot.plot(('n', 'e'), type='scatter', color='blue', marker='plus')
    self.plot.plot(('ref_n', 'ref_e'),
        type='scatter',
        color='red',
        marker='cross',
        marker_size=10,
        line_width=1.5
    )
    #self.plot.plot(('h', 'e'), type='line', color='red')

    self.pr_plot_data = ArrayPlotData(t=[0.0])
    self.pr_plot = Plot(self.pr_plot_data, auto_colors=colours_list)
    self.pr_plot.value_range.tight_bounds = False
    #self.pr_plot.value_range.low_setting = 0.0
    for n in range(TRACK_N_CHANNELS):
      self.pr_plot_data.set_data('prs'+str(n), [0.0])
      self.pr_plot.plot(('t', 'prs'+str(n)), type='line', color='auto')
      #self.pr_plot_data.set_data('pred_prs'+str(n), [0.0])
      #self.pr_plot.plot(('t', 'pred_prs'+str(n)), type='line', color='auto')

    self.python_console_cmds = {
      'solution': self
    }


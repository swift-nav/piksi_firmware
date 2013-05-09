from traits.api import Str, Instance, Dict, HasTraits, Array, Float, on_trait_change, List, Int
from traitsui.api import Item, View, HGroup, ArrayEditor

from chaco.api import BarPlot, ArrayDataSource, DataRange1D, LinearMapper, OverlayPlotContainer, LabelAxis, PlotAxis, ArrayPlotData, Plot, Legend
from chaco.tools.api import LegendTool
from enable.api import ComponentEditor, Component

from pyface.api import GUI

import struct

import numpy as np

MSG_TRACKING_STATE = 0x22
TRACK_N_CHANNELS = 12

colours_list = [
  0xE41A1C,
  0x377EB8,
  0x4DAF4A,
  0x984EA3,
  0xFF7F00,
  0xFFFF33,
  0xA65628,
  0xF781BF,
]

class TrackingState(HasTraits):
  state = Int()
  cn0 = Float()
  prn = Int()

  def __init__(self, *args, **kwargs):
    self.update(*args, **kwargs)

  def update(self, state, prn, cn0):
    self.state = state
    self.cn0 = np.nan if cn0 == -1 else cn0
    self.prn = prn

  def __repr__(self):
    return "TS: %d %f" % (self.prn, self.cn0)

class TrackingView(HasTraits):
  python_console_cmds = Dict()
  states = List(Instance(TrackingState))
  cn0_history = List()

  plot = Instance(Plot)
  plots = List()
  plot_data = Instance(ArrayPlotData)

  traits_view = View(
    HGroup(
      Item(
        'plot',
        editor = ComponentEditor(bgcolor = (0.8, 0.8, 0.8)),
        show_label = False,
      )
    )
  )

  def tracking_state_callback(self, data):
    fmt = '<' + TRACK_N_CHANNELS * 'BBf'
    state_data = struct.unpack(fmt, data)
    for n, s in enumerate(self.states):
      s.update(*state_data[3*n:3*(n+1)])
    GUI.invoke_later(self.update_plot)

  def update_plot(self):
    self.cn0_history.append([s.cn0 for s in self.states])
    self.cn0_history = self.cn0_history[-1000:]

    chans = np.transpose(self.cn0_history[-200:])
    t = range(len(chans[0]))
    self.plot_data.set_data('t', t)
    plot_labels = []
    for n in range(TRACK_N_CHANNELS):
      self.plot_data.set_data('ch'+str(n), chans[n])
      if self.states[n].state == 0:
        plot_labels.append('Ch %02d (Disabled)' % n)
      else:
        plot_labels.append('Ch %02d (PRN%02d)' % (n, self.states[n].prn+1))
    plots = dict(zip(plot_labels, self.plots))
    self.plot.legend.plots = plots

  def __init__(self, link):
    super(TrackingView, self).__init__()

    self.states = [TrackingState(0, 0, 0) for _ in range(TRACK_N_CHANNELS)]

    self.link = link
    self.link.add_callback(MSG_TRACKING_STATE, self.tracking_state_callback)

    self.plot_data = ArrayPlotData(t=[0.0])
    self.plot = Plot(self.plot_data, auto_colors=colours_list)
    self.plot.title = "Tracking C/N0"
    self.plot.value_range.margin = 0.1
    self.plot.value_range.bounds_func = lambda l, h, m, tb: (0, h*(1+m))
    self.plot.value_axis.orientation = 'right'
    self.plot.value_axis.axis_line_visible = False
    for n in range(TRACK_N_CHANNELS):
      self.plot_data.set_data('ch'+str(n), [0.0])
      pl = self.plot.plot(('t', 'ch'+str(n)), type='line', color='auto')
      self.plots.append(pl)
    self.plot.legend.visible = True
    self.plot.legend.align = 'ul'
    self.plot.legend.tools.append(LegendTool(self.plot.legend, drag_button="right"))
    self.update_plot()

    self.python_console_cmds = {
      'track': self
    }


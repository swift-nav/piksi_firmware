#!/usr/bin/env python
# Copyright (C) 2011-2014 Swift Navigation Inc.
# Contact: Fergus Noble <fergus@swift-nav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

from traits.api import Str, Instance, Dict, HasTraits, Array, Float, on_trait_change, List, Int
from traitsui.api import Item, View, HGroup, ArrayEditor, HSplit

from chaco.api import BarPlot, ArrayDataSource, DataRange1D, LinearMapper, OverlayPlotContainer, LabelAxis, PlotAxis, ArrayPlotData, Plot, Legend
from chaco.tools.api import LegendTool
from enable.api import ComponentEditor, Component

from pyface.api import GUI

import struct

import numpy as np

import sbp_piksi as ids

TRACKING_STATE_BYTES_PER_CHANNEL = 6
NUM_POINTS = 200

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
    HSplit(
      Item(
        'plot',
        editor = ComponentEditor(bgcolor = (0.8, 0.8, 0.8)),
        show_label = False,
      )
    )
  )

  def tracking_state_callback(self, data):
    n_channels = len(data) / TRACKING_STATE_BYTES_PER_CHANNEL

    if n_channels != self.n_channels:
      # Update number of channels
      self.n_channels = n_channels
      self.states = [TrackingState(0, 0, 0) for _ in range(n_channels)]
      for pl in self.plot.plots.iterkeys():
        self.plot.delplot(pl.name)
      self.plots = []
      for n in range(n_channels):
        self.plot_data.set_data('ch'+str(n), [0.0])
        pl = self.plot.plot(('t', 'ch'+str(n)), type='line', color='auto', name='ch'+str(n))
        self.plots.append(pl)
      print 'Number of tracking channels changed to', n_channels

    fmt = '<' + n_channels * 'BBf'
    state_data = struct.unpack(fmt, data)
    for n, s in enumerate(self.states):
      s.update(*state_data[3*n:3*(n+1)])
    GUI.invoke_later(self.update_plot)

  def update_plot(self):
    self.cn0_history.append([s.cn0 for s in self.states])
    self.cn0_history = self.cn0_history[-1000:]

    chans = np.transpose(self.cn0_history[-NUM_POINTS:])
    plot_labels = []
    for n in range(self.n_channels):
      self.plot_data.set_data('ch'+str(n), chans[n])
      if self.states[n].state == 0:
        plot_labels.append('Ch %02d (Disabled)' % n)
      else:
        plot_labels.append('Ch %02d (PRN%02d)' % (n, self.states[n].prn+1))
    plots = dict(zip(plot_labels, self.plots))
    self.plot.legend.plots = plots

  def __init__(self, link):
    super(TrackingView, self).__init__()

    self.n_channels = None

    self.plot_data = ArrayPlotData(t=[0.0])
    self.plot = Plot(self.plot_data, auto_colors=colours_list)
    self.plot.title = "Tracking C/N0"
    self.plot.value_range.margin = 0.1
    self.plot.value_range.bounds_func = lambda l, h, m, tb: (0, h*(1+m))
    self.plot.value_axis.orientation = 'right'
    self.plot.value_axis.axis_line_visible = False
    t = range(NUM_POINTS)
    self.plot_data.set_data('t', t)

    self.plot.legend.visible = True
    self.plot.legend.align = 'ul'
    self.plot.legend.tools.append(LegendTool(self.plot.legend, drag_button="right"))

    self.link = link
    self.link.add_callback(ids.TRACKING_STATE, self.tracking_state_callback)

    self.python_console_cmds = {
      'track': self
    }


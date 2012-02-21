from enthought.traits.api import Str, Instance, Dict, HasTraits, Array, Float, on_trait_change
from enthought.traits.ui.api import Item, View

from enthought.chaco.api import BarPlot, ArrayDataSource, DataRange1D, LinearMapper, OverlayPlotContainer, LabelAxis, PlotAxis
from enthought.enable.api import ComponentEditor, Component

import struct

import numpy as n

MSG_TRACKING_SNRS = 0x22
TRACK_N_CHANNELS = 4

class TrackingView(HasTraits):
  python_console_cmds = Dict()
  snrs = Array(dtype=float, shape=(TRACK_N_CHANNELS,))
  snr_bars = Instance(Component)

  snr_bar_view = View(
    Item('snr_bars', editor=ComponentEditor(size=(100,100)), show_label=False)
  )

  traits_view = View(
    Item('snrs')
  )

  def tracking_snrs_callback(self, data):
    fmt = '<' + str(TRACK_N_CHANNELS) + 'f'
    self.snrs = struct.unpack(fmt, data)

  @on_trait_change('snrs')
  def update_snr_bars(self):
    self.vals.set_data(self.snrs)

  def __init__(self, link):
    super(TrackingView, self).__init__()

    self.link = link
    self.link.add_callback(MSG_TRACKING_SNRS, self.tracking_snrs_callback)

    idxs = ArrayDataSource(range(1, len(self.snrs)+1))
    self.vals = ArrayDataSource(self.snrs, sort_order='none')

    # Create the index range
    index_range = DataRange1D(idxs, low=0.4, high=TRACK_N_CHANNELS+0.6)
    index_mapper = LinearMapper(range=index_range)
    # Create the value range
    value_range = DataRange1D(low=0.0, high=25.0)
    value_mapper = LinearMapper(range=value_range)

    plot = BarPlot(index=idxs, value=self.vals, 
                   index_mapper=index_mapper, value_mapper=value_mapper, 
                   line_color='black', fill_color='lightgreen', bar_width=0.8)

    container = OverlayPlotContainer(bgcolor = "white")
    plot.padding = 10
    plot.padding_left = 30
    plot.padding_bottom = 30
    container.add(plot)

    left_axis = PlotAxis(plot, orientation='left')
    bottom_axis = LabelAxis(plot, orientation='bottom',
                           labels = ['1', '2', '3', '4'],
                           positions = range(1, TRACK_N_CHANNELS+1),
                           small_haxis_style=True)

    plot.underlays.append(left_axis)
    plot.underlays.append(bottom_axis)

    self.snr_bars = container

    self.python_console_cmds = {
      'track': self
    }


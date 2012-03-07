from enthought.traits.api import Str, Instance, Dict, HasTraits, Array, Float, on_trait_change, List
from enthought.traits.ui.api import Item, View

from enthought.chaco.api import BarPlot, ArrayDataSource, DataRange1D, LinearMapper, OverlayPlotContainer, LabelAxis, PlotAxis, ArrayPlotData, Plot
from enthought.enable.api import ComponentEditor, Component

from enthought.pyface.api import GUI

import struct

import numpy as np

MSG_TRACKING_SNRS = 0x22
TRACK_N_CHANNELS = 5

colours_list = ['red', 'blue', 'green', 'purple']

class TrackingView(HasTraits):
  python_console_cmds = Dict()
  snrs = Array(dtype=float, shape=(TRACK_N_CHANNELS,))
  snrs_avg = Array(dtype=float, shape=(TRACK_N_CHANNELS,))
  snrs_history = List()
  snr_bars = Instance(Component)

  plot = Instance(Plot)
  plot_data = Instance(ArrayPlotData)

  snr_bar_view = View(
    Item('snr_bars', editor=ComponentEditor(size=(100,100)), show_label=False)
  )

  snr_line_view = View(
    Item(
      'plot',
      editor = ComponentEditor(bgcolor = (0.8,0.8,0.8)),
      show_label = False,
    )
  )

  traits_view = View(
    Item('snrs'),
    Item('snrs_avg')
  )

  def tracking_snrs_callback(self, data):
    fmt = '<' + str(TRACK_N_CHANNELS) + 'f'
    self.snrs = struct.unpack(fmt, data)

  def _snrs_changed(self):
    GUI.invoke_later(self.update_snrs)

  def update_snrs(self):
    self.snrs_history.append(self.snrs)
    s = np.array(self.snrs_history[-100:])
    self.snrs_avg = np.sum(s, axis=0) / 100
    # Filter out channels that are not tracking.
    #self.snrs_avg = map(lambda n, x: x if self.snrs[n] != -1 else -1, self.snrs_avg)
    self.vals.set_data(self.snrs)

    chans = np.transpose(self.snrs_history[-500:])
    t = range(len(chans[0]))
    self.plot_data.set_data('t', t)
    for n in range(TRACK_N_CHANNELS):
      self.plot_data.set_data('ch'+str(n), chans[n])

  def __init__(self, link):
    super(TrackingView, self).__init__()

    self.link = link
    self.link.add_callback(MSG_TRACKING_SNRS, self.tracking_snrs_callback)

    # ======= Line Plot =======

    self.plot_data = ArrayPlotData(t=[0.0])
    self.plot = Plot(self.plot_data, auto_colors=colours_list)
    self.plot.value_range.tight_bounds = False
    self.plot.value_range.low_setting = 0.0
    for n in range(TRACK_N_CHANNELS):
      self.plot_data.set_data('ch'+str(n), [0.0])
      self.plot.plot(('t', 'ch'+str(n)), type='line', color='auto')

    # ======= Bar Plot =======

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
                   line_color='blue', fill_color='blue', bar_width=0.8)

    container = OverlayPlotContainer(bgcolor = "white")
    plot.padding = 10
    plot.padding_left = 30
    plot.padding_bottom = 30
    container.add(plot)

    left_axis = PlotAxis(plot, orientation='left')
    bottom_axis = LabelAxis(plot, orientation='bottom',
                           labels = map(str, range(1, TRACK_N_CHANNELS+1)),
                           positions = range(1, TRACK_N_CHANNELS+1),
                           small_haxis_style=True)

    plot.underlays.append(left_axis)
    plot.underlays.append(bottom_axis)

    self.snr_bars = container

    self.python_console_cmds = {
      'track': self
    }


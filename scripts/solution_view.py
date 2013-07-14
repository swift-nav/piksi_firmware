from traits.api import Instance, Dict, HasTraits, Array, Float, on_trait_change, List, Int, Button, Bool
from traitsui.api import Item, View, HGroup, VGroup, ArrayEditor, HSplit, TabularEditor
from traitsui.tabular_adapter import TabularAdapter
from chaco.api import ArrayPlotData, Plot
from chaco.tools.api import ZoomTool, PanTool
from enable.api import ComponentEditor
from enable.savage.trait_defs.ui.svg_button import SVGButton
from pyface.api import GUI

import struct
import math
import os
import numpy as np
import datetime

MSG_SOLUTION = 0x50
MSG_SOLUTION_DOPS = 0x51

class SimpleAdapter(TabularAdapter):
    columns = [('Item', 0), ('Value',  1)]
    width = 80

class Solution:
  def from_binary(self, data):
    soln = struct.unpack('<3d3d3d3d7ddddHBB', data)
    self.pos_llh = [soln[0]*(180/math.pi), soln[1]*(180/math.pi), soln[2]]
    self.pos_ecef = soln[3:6]
    self.vel_ned = soln[6:9]
    self.vel_ecef = soln[9:12]
    self.err_cov = soln[12:19]
    self.clock_offset = soln[19]
    self.clock_bias = soln[20]
    self.gps_tow = soln[21]
    self.gps_week = soln[22]
    self.soln_valid = soln[23]
    self.n_used = soln[24]


class SolutionView(HasTraits):
  python_console_cmds = Dict()

  lats = List()
  lngs = List()
  alts = List()

  table = List()
  dops_table = List()
  soln_table = List()

  plot = Instance(Plot)
  plot_data = Instance(ArrayPlotData)

  running = Bool(True)
  position_centered = Bool(False)

  clear_button = SVGButton(
    label='', tooltip='Clear',
    filename=os.path.join(os.path.dirname(__file__), 'images', 'x.svg'),
    width=16, height=16
  )
  zoomall_button = SVGButton(
    label='', tooltip='Zoom All',
    filename=os.path.join(os.path.dirname(__file__), 'images', 'fullscreen.svg'),
    width=16, height=16
  )
  center_button = SVGButton(
    label='', tooltip='Center on Solution', toggle=True,
    filename=os.path.join(os.path.dirname(__file__), 'images', 'target.svg'),
    width=16, height=16
  )
  paused_button = SVGButton(
    label='', tooltip='Pause', toggle_tooltip='Run', toggle=True,
    filename=os.path.join(os.path.dirname(__file__), 'images', 'pause.svg'),
    toggle_filename=os.path.join(os.path.dirname(__file__), 'images', 'play.svg'),
    width=16, height=16
  )

  traits_view = View(
    HSplit(
      Item('table', style = 'readonly', editor = TabularEditor(adapter=SimpleAdapter()), show_label=False, width=0.3),
      VGroup(
        HGroup(
          Item('paused_button', show_label=False),
          Item('clear_button', show_label=False),
          Item('zoomall_button', show_label=False),
          Item('center_button', show_label=False),
        ),
        Item(
          'plot',
          show_label = False,
          editor = ComponentEditor(bgcolor = (0.8,0.8,0.8)),
        )
      )
    )
  )

  def _zoomall_button_fired(self):
    self.plot.index_range.low_setting = 'auto'
    self.plot.index_range.high_setting = 'auto'
    self.plot.value_range.low_setting = 'auto'
    self.plot.value_range.high_setting = 'auto'

  def _center_button_fired(self):
    self.position_centered = not self.position_centered

  def _paused_button_fired(self):
    self.running = not self.running

  def _clear_button_fired(self):
    self.lats = []
    self.lngs = []
    self.alts = []
    self.plot_data.set_data('lat', [])
    self.plot_data.set_data('lng', [])
    self.plot_data.set_data('alt', [])
    self.plot_data.set_data('t', [])

  def _solution_callback(self, data):
    # Updating an ArrayPlotData isn't thread safe (see chaco issue #9), so
    # actually perform the update in the UI thread.
    if self.running:
      GUI.invoke_later(self.solution_callback, data)

  def update_table(self):
    self._table_list = self.table.items()

  def solution_callback(self, data):
    soln = Solution()
    soln.from_binary(data)
    self.soln_table = []

    t = datetime.datetime(1980, 1, 5) + \
        datetime.timedelta(weeks=soln.gps_week) + \
        datetime.timedelta(seconds=soln.gps_tow)
    self.soln_table.append(('GPS Time', str(t)))

    self.soln_table.append(('Lat', soln.pos_llh[0]))
    self.soln_table.append(('Lng', soln.pos_llh[1]))
    self.soln_table.append(('Alt', soln.pos_llh[2]))

    self.soln_table.append(('Vel. N', soln.vel_ned[0]))
    self.soln_table.append(('Vel. E', soln.vel_ned[1]))
    self.soln_table.append(('Vel. D', soln.vel_ned[2]))

    self.soln_table.append(('GPS ToW', soln.gps_tow))
    self.soln_table.append(('GPS Week', soln.gps_week))
    self.soln_table.append(('Num. sats', soln.n_used))

    self.lats.append(soln.pos_llh[0])
    self.lngs.append(soln.pos_llh[1])
    self.alts.append(soln.pos_llh[2])

    self.plot_data.set_data('lat', self.lats)
    self.plot_data.set_data('lng', self.lngs)
    self.plot_data.set_data('alt', self.alts)
    t = range(len(self.lats))
    self.plot_data.set_data('t', t)

    self.table = self.soln_table + self.dops_table

    if self.position_centered:
      d = (self.plot.index_range.high - self.plot.index_range.low) / 2.
      self.plot.index_range.set_bounds(soln.pos_llh[0] - d, soln.pos_llh[0] + d)
      d = (self.plot.value_range.high - self.plot.value_range.low) / 2.
      self.plot.value_range.set_bounds(soln.pos_llh[1] - d, soln.pos_llh[1] + d)

  def dops_callback(self, data):
    p, g, t, h, v = struct.unpack('<ddddd', data)
    self.dops_table = [
      ('PDOP', '%.1f' % p),
      ('GDOP', '%.1f' % g),
      ('TDOP', '%.1f' % t),
      ('HDOP', '%.1f' % h),
      ('VDOP', '%.1f' % v)
    ]
    self.table = self.soln_table + self.dops_table

  def __init__(self, link):
    super(SolutionView, self).__init__()

    self.plot_data = ArrayPlotData(lat=[0.0], lng=[0.0], alt=[0.0], t=[0.0], ref_lat=[0.0], ref_lng=[0.0], region_lat=[0.0], region_lng=[0.0])
    self.plot = Plot(self.plot_data)

    self.plot.plot(('lat', 'lng'), type='line', name='line', color=(0, 0, 0, 0.1))
    self.plot.plot(('lat', 'lng'), type='scatter', name='points', color='blue', marker='dot', marker_size=1.0)

    self.plot.index_axis.tick_label_position = 'inside'
    self.plot.index_axis.tick_label_color = 'gray'
    self.plot.index_axis.tick_color = 'gray'
    self.plot.value_axis.tick_label_position = 'inside'
    self.plot.value_axis.tick_label_color = 'gray'
    self.plot.value_axis.tick_color = 'gray'
    self.plot.padding = (0, 1, 0, 1)

    self.plot.tools.append(PanTool(self.plot))
    zt = ZoomTool(self.plot, zoom_factor=1.1, tool_mode="box", always_on=False)
    self.plot.overlays.append(zt)

    self.link = link
    self.link.add_callback(MSG_SOLUTION, self._solution_callback)
    self.link.add_callback(MSG_SOLUTION_DOPS, self.dops_callback)

    self.python_console_cmds = {
      'solution': self
    }


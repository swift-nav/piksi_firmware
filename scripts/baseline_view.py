# Copyright (C) 2011-2014 Swift Navigation Inc.
# Contact: Fergus Noble <fergus@swift-nav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

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
import time

from sbp.piksi      import SBP_MSG_IAR_STATE, SBP_MSG_INIT_BASE, SBP_MSG_RESET_FILTERS
from sbp.navigation import SBP_MSG_BASELINE_ECEF, SBP_MSG_BASELINE_NED, SBP_MSG_GPS_TIME, MsgBaselineNED, MsgGPSTime

class SimpleAdapter(TabularAdapter):
    columns = [('Item', 0), ('Value',  1)]
    width = 80

class Baseline:
  def from_binary(self, data):
    soln = struct.unpack('<3ddHHB', data)
    self.ned = np.array([soln[0], soln[1], soln[2]])
    self.tow = soln[3] / 1e3
    self.wn = soln[4]
    self.flags = soln[5]
    self.n_sats = soln[6]

class BaselineView(HasTraits):
  python_console_cmds = Dict()

  ns = List()
  es = List()
  ds = List()

  table = List()

  plot = Instance(Plot)
  plot_data = Instance(ArrayPlotData)

  running = Bool(True)
  position_centered = Bool(False)

  clear_button = SVGButton(
    label='', tooltip='Clear',
    filename=os.path.join(os.path.dirname(__file__), 'images', 'iconic', 'x.svg'),
    width=16, height=16
  )
  zoomall_button = SVGButton(
    label='', tooltip='Zoom All',
    filename=os.path.join(os.path.dirname(__file__), 'images', 'iconic', 'fullscreen.svg'),
    width=16, height=16
  )
  center_button = SVGButton(
    label='', tooltip='Center on Baseline', toggle=True,
    filename=os.path.join(os.path.dirname(__file__), 'images', 'iconic', 'target.svg'),
    width=16, height=16
  )
  paused_button = SVGButton(
    label='', tooltip='Pause', toggle_tooltip='Run', toggle=True,
    filename=os.path.join(os.path.dirname(__file__), 'images', 'iconic', 'pause.svg'),
    toggle_filename=os.path.join(os.path.dirname(__file__), 'images', 'iconic', 'play.svg'),
    width=16, height=16
  )

  reset_button = Button(label='Reset Filters')
  reset_iar_button = Button(label='Reset IAR')
  init_base_button = Button(label='Init. with known baseline')

  traits_view = View(
    HSplit(
      Item('table', style = 'readonly', editor = TabularEditor(adapter=SimpleAdapter()), show_label=False, width=0.3),
      VGroup(
        HGroup(
          Item('paused_button', show_label=False),
          Item('clear_button', show_label=False),
          Item('zoomall_button', show_label=False),
          Item('center_button', show_label=False),
          Item('reset_button', show_label=False),
          Item('reset_iar_button', show_label=False),
          Item('init_base_button', show_label=False),
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

  def _reset_button_fired(self):
    self.link.send_message(SBP_MSG_RESET_FILTERS, '\x00')

  def _reset_iar_button_fired(self):
    self.link.send_message(SBP_MSG_RESET_FILTERS, '\x01')

  def _init_base_button_fired(self):
    self.link.send_message(SBP_MSG_INIT_BASE, '')

  def _clear_button_fired(self):
    self.ns = []
    self.es = []
    self.ds = []
    self.plot_data.set_data('n', [])
    self.plot_data.set_data('e', [])
    self.plot_data.set_data('d', [])
    self.plot_data.set_data('t', [])
    self.plot_data.set_data('curr_n', [])
    self.plot_data.set_data('curr_e', [])
    self.plot_data.set_data('curr_d', [])

  def _baseline_callback_ecef(self, data):
    #Don't do anything for ECEF currently
    return

  def iar_state_callback(self, data):
    self.num_hyps = struct.unpack('<I', data.payload)[0]

  def _baseline_callback_ned(self, data):
    # Updating an ArrayPlotData isn't thread safe (see chaco issue #9), so
    # actually perform the update in the UI thread.
    if self.running:
      GUI.invoke_later(self.baseline_callback, data)

  def update_table(self):
    self._table_list = self.table.items()

  def gps_time_callback(self, data):
    self.week = MsgGPSTime(data).wn
    self.nsec = MsgGPSTime(data).ns

  def baseline_callback(self, data):
    soln = MsgBaselineNED(data)
    table = []

    soln.n = soln.n * 1e-3
    soln.e = soln.e * 1e-3
    soln.d = soln.d * 1e-3

    dist = np.sqrt(soln.n**2 + soln.e**2 + soln.d**2)

    tow = soln.tow * 1e-3
    if self.nsec is not None:
      tow += self.nsec * 1e-9

    if self.week is not None:
      t = datetime.datetime(1980, 1, 6) + \
          datetime.timedelta(weeks=self.week) + \
          datetime.timedelta(seconds=tow)

      table.append(('GPS Time', t))
      table.append(('GPS Week', str(self.week)))

      if self.log_file is None:
        self.log_file = open(time.strftime("baseline_log_%Y%m%d-%H%M%S.csv"), 'w')

      self.log_file.write('%s,%.4f,%.4f,%.4f,%.4f,%d,0x%02x,%d\n' % (
        str(t),
        soln.n, soln.e, soln.d, dist,
        soln.n_sats,
        soln.flags,
        self.num_hyps)
      )
      self.log_file.flush()

    table.append(('GPS ToW', tow))

    table.append(('N', soln.n))
    table.append(('E', soln.e))
    table.append(('D', soln.d))
    table.append(('Dist.', dist))
    table.append(('Num. Sats.', soln.n_sats))
    table.append(('Flags', '0x%02x' % soln.flags))
    if soln.flags & 1:
      table.append(('Mode', 'Fixed RTK'))
    else:
      table.append(('Mode', 'Float'))
    table.append(('IAR Num. Hyps.', self.num_hyps))

    self.ns.append(soln.n)
    self.es.append(soln.e)
    self.ds.append(soln.d)

    self.ns = self.ns[-1000:]
    self.es = self.es[-1000:]
    self.ds = self.ds[-1000:]

    self.plot_data.set_data('n', self.ns)
    self.plot_data.set_data('e', self.es)
    self.plot_data.set_data('d', self.ds)
    self.plot_data.set_data('curr_n', [soln.n])
    self.plot_data.set_data('curr_e', [soln.e])
    self.plot_data.set_data('curr_d', [soln.d])
    self.plot_data.set_data('ref_n', [0.0])
    self.plot_data.set_data('ref_e', [0.0])
    self.plot_data.set_data('ref_d', [0.0])

    t = range(len(self.ns))
    self.plot_data.set_data('t', t)

    if self.position_centered:
      d = (self.plot.index_range.high - self.plot.index_range.low) / 2.
      self.plot.index_range.set_bounds(soln.e - d, soln.e + d)
      d = (self.plot.value_range.high - self.plot.value_range.low) / 2.
      self.plot.value_range.set_bounds(soln.n - d, soln.n + d)

    self.table = table

  def __init__(self, link):
    super(BaselineView, self).__init__()

    self.log_file = None

    self.num_hyps = 0

    self.plot_data = ArrayPlotData(n=[0.0], e=[0.0], d=[0.0], t=[0.0], ref_n=[0.0],
                     ref_e=[0.0], ref_d=[0.0], curr_e=[0.0], curr_n=[0.0], curr_d=[0.0])
    self.plot = Plot(self.plot_data)
    lin = self.plot.plot(('e', 'n'),
        type='line',
        color=(0, 0, 1, 0.1))
    pts = self.plot.plot(('e', 'n'),
        type='scatter',
        color='blue',
        marker='dot',
        line_width=0.0,
        marker_size=1.0)
    ref = self.plot.plot(('ref_e', 'ref_n'),
        type='scatter',
        color='red',
        marker='plus',
        marker_size=5,
        line_width=1.5)
    cur = self.plot.plot(('curr_e', 'curr_n'),
        type='scatter',
        color='orange',
        marker='plus',
        marker_size=5,
        line_width=1.5)
    plot_labels = ['Base Position','Rover Relative Position','Rover Path']
    plots_legend = dict(zip(plot_labels, [ref,cur,pts]))
    self.plot.legend.plots = plots_legend
    self.plot.legend.visible = True

    self.plot.index_axis.tick_label_position = 'inside'
    self.plot.index_axis.tick_label_color = 'gray'
    self.plot.index_axis.tick_color = 'gray'
    self.plot.index_axis.title='E (meters)'
    self.plot.index_axis.title_spacing = 5
    self.plot.value_axis.tick_label_position = 'inside'
    self.plot.value_axis.tick_label_color = 'gray'
    self.plot.value_axis.tick_color = 'gray'
    self.plot.value_axis.title='N (meters)'
    self.plot.value_axis.title_spacing = 5
    self.plot.padding = (25, 25, 25, 25)

    self.plot.tools.append(PanTool(self.plot))
    zt = ZoomTool(self.plot, zoom_factor=1.1, tool_mode="box", always_on=False)
    self.plot.overlays.append(zt)

    self.week = None
    self.nsec = 0

    self.link = link
    self.link.add_callback(SBP_MSG_BASELINE_NED, self._baseline_callback_ned)
    self.link.add_callback(SBP_MSG_BASELINE_ECEF, self._baseline_callback_ecef)
    self.link.add_callback(SBP_MSG_IAR_STATE, self.iar_state_callback)
    self.link.add_callback(SBP_MSG_GPS_TIME, self.gps_time_callback)

    self.python_console_cmds = {
      'baseline': self
    }

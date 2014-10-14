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

from traits.api import Instance, Dict, HasTraits, Array, Float, on_trait_change, List, Int, Button, Bool
from traitsui.api import Item, View, HGroup, VGroup, ArrayEditor, HSplit, TabularEditor, UItem
from traitsui.tabular_adapter import TabularAdapter
from chaco.api import ArrayPlotData, Plot
from chaco.tools.api import ZoomTool, PanTool
from enable.api import ComponentEditor
from enable.savage.trait_defs.ui.svg_button import SVGButton
from pyface.api import GUI

from simulator_view import SimulatorView

import struct
import math
import os
import numpy as np
import datetime
import time

import sbp_piksi as sbp_messages

class SimpleAdapter(TabularAdapter):
    columns = [('Item', 0), ('Value',  1)]
    width = 80

class SolutionView(HasTraits):
  python_console_cmds = Dict()

  lats = List()
  lngs = List()
  alts = List()

  table = List()
  dops_table = List()
  pos_table = List()
  vel_table = List()

  plot = Instance(Plot)
  plot_data = Instance(ArrayPlotData)

  simulator_view = Instance(SimulatorView)

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
    label='', tooltip='Center on Solution', toggle=True,
    filename=os.path.join(os.path.dirname(__file__), 'images', 'iconic', 'target.svg'),
    width=16, height=16
  )
  paused_button = SVGButton(
    label='', tooltip='Pause', toggle_tooltip='Run', toggle=True,
    filename=os.path.join(os.path.dirname(__file__), 'images', 'iconic', 'pause.svg'),
    toggle_filename=os.path.join(os.path.dirname(__file__), 'images', 'iconic', 'play.svg'),
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
        ),
        UItem('simulator_view', style='custom', label='Simulator'),
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

  def _pos_llh_callback(self, data):
    # Updating an ArrayPlotData isn't thread safe (see chaco issue #9), so
    # actually perform the update in the UI thread.
    if self.running:
      GUI.invoke_later(self.pos_llh_callback, data)

  def update_table(self):
    self._table_list = self.table.items()

  def pos_llh_callback(self, data):
    soln = sbp_messages.PosLLH(data)
    self.pos_table = []

    if self.log_file is None:
      self.log_file = open(time.strftime("position_log_%Y%m%d-%H%M%S.csv"), 'w')

    tow = soln.tow * 1e-3
    if self.nsec is not None:
      tow += self.nsec * 1e-9

    if self.week is not None:
      t = datetime.datetime(1980, 1, 6) + \
          datetime.timedelta(weeks=self.week) + \
          datetime.timedelta(seconds=tow)
      self.pos_table.append(('GPS Time', t))
      self.pos_table.append(('GPS Week', str(self.week)))

      self.log_file.write('%s,%.10f,%.10f,%.4f,%d\n' % (
        str(t),
        soln.lat, soln.lon, soln.height,
        soln.n_sats)
      )
      self.log_file.flush()

    self.pos_table.append(('GPS ToW', tow))

    self.pos_table.append(('Num. sats', soln.n_sats))

    self.pos_table.append(('Lat', soln.lat))
    self.pos_table.append(('Lng', soln.lon))
    self.pos_table.append(('Alt', soln.height))

    self.lats.append(soln.lat)
    self.lngs.append(soln.lon)
    self.alts.append(soln.height)

    self.lats = self.lats[-1000:]
    self.lngs = self.lngs[-1000:]
    self.alts = self.alts[-1000:]

    self.plot_data.set_data('lat', self.lats)
    self.plot_data.set_data('lng', self.lngs)
    self.plot_data.set_data('alt', self.alts)
    t = range(len(self.lats))
    self.plot_data.set_data('t', t)

    self.table = self.pos_table + self.vel_table + self.dops_table

    if self.position_centered:
      d = (self.plot.index_range.high - self.plot.index_range.low) / 2.
      self.plot.index_range.set_bounds(soln.lon - d, soln.lon + d)
      d = (self.plot.value_range.high - self.plot.value_range.low) / 2.
      self.plot.value_range.set_bounds(soln.lat - d, soln.lat + d)

  def dops_callback(self, data):
    dops = sbp_messages.Dops(data)
    self.dops_table = [
      ('PDOP', '%.1f' % (dops.pdop * 0.01)),
      ('GDOP', '%.1f' % (dops.gdop * 0.01)),
      ('TDOP', '%.1f' % (dops.tdop * 0.01)),
      ('HDOP', '%.1f' % (dops.hdop * 0.01)),
      ('VDOP', '%.1f' % (dops.vdop * 0.01))
    ]
    self.table = self.pos_table + self.vel_table + self.dops_table

  def vel_ned_callback(self, data):
    vel_ned = sbp_messages.VelNED(data)

    if self.vel_log_file is None:
      self.vel_log_file = open(time.strftime("velocity_log_%Y%m%d-%H%M%S.csv"), 'w')

    tow = vel_ned.tow * 1e-3
    if self.nsec is not None:
      tow += self.nsec * 1e-9

    if self.week is not None:
      t = datetime.datetime(1980, 1, 6) + \
          datetime.timedelta(weeks=self.week) + \
          datetime.timedelta(seconds=tow)

      self.vel_log_file.write('%s,%.6f,%.6f,%.6f,%.6f,%d\n' % (
        str(t),
        vel_ned.n * 1e-3, vel_ned.e * 1e-3, vel_ned.d * 1e-3,
        math.sqrt(vel_ned.n*vel_ned.n + vel_ned.e*vel_ned.e) * 1e-3,
        vel_ned.n_sats)
      )
      self.vel_log_file.flush()

    self.vel_table = [
      ('Vel. N', '% 8.4f' % (vel_ned.n * 1e-3)),
      ('Vel. E', '% 8.4f' % (vel_ned.e * 1e-3)),
      ('Vel. D', '% 8.4f' % (vel_ned.d * 1e-3)),
    ]
    self.table = self.pos_table + self.vel_table + self.dops_table

  def gps_time_callback(self, data):
    self.week = sbp_messages.GPSTime(data).wn
    self.nsec = sbp_messages.GPSTime(data).ns

  def __init__(self, link):
    super(SolutionView, self).__init__()

    self.log_file = None
    self.vel_log_file = None

    self.plot_data = ArrayPlotData(lat=[0.0], lng=[0.0], alt=[0.0], t=[0.0], ref_lat=[0.0], ref_lng=[0.0], region_lat=[0.0], region_lng=[0.0])
    self.plot = Plot(self.plot_data)

    self.plot.plot(('lng', 'lat'), type='line', name='line', color=(0, 0, 0, 0.1))
    self.plot.plot(('lng', 'lat'), type='scatter', name='points', color='blue', marker='dot', line_width=0.0, marker_size=1.0)

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
    self.link.add_callback(sbp_messages.SBP_POS_LLH, self._pos_llh_callback)
    self.link.add_callback(sbp_messages.SBP_VEL_NED, self.vel_ned_callback)
    self.link.add_callback(sbp_messages.SBP_DOPS, self.dops_callback)
    self.link.add_callback(sbp_messages.SBP_GPS_TIME, self.gps_time_callback)

    self.week = None
    self.nsec = 0

    self.simulator_view = SimulatorView(self.link)

    self.python_console_cmds = {
      'solution': self,
      'simulator': self.simulator_view
    }


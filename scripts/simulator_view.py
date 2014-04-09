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
from traitsui.api import Item, View, HGroup, VGroup, ArrayEditor, HSplit, TabularEditor
from traitsui.tabular_adapter import TabularAdapter

import struct
import math
import os
import numpy as np
import datetime

import sbp_piksi as ids


class SimulationSettings(HasTraits):
  """
  SBP class for message SIMULATION_SETTINGS (0x0206)

  """

  def __init__(self, d = None, l = None):
    if d is not None:
      self.from_binary(d)
    elif l is not None:
      self.from_list(l)
    else:
      self.base_ecef_x = 0.0
      self.base_ecef_y = 0.0
      self.base_ecef_z = 0.0
      self.speed = 0.0
      self.radius = 0.0
      self.pos_variance = 0.0
      self.speed_variance = 0.0
      self.tracking_cn0_variance = 0.0
      self.pseudorange_variance = 0.0
      self.carrier_phase_variance = 0.0
      self.num_sats = 0.0
      self.enabled = 0
      self.mode_mask = 0

  def from_binary(self, d):
    (
      self.base_ecef_x,
      self.base_ecef_y,
      self.base_ecef_z,
      self.speed,
      self.radius,
      self.pos_variance,
      self.speed_variance,
      self.tracking_cn0_variance,
      self.pseudorange_variance,
      self.carrier_phase_variance,
      self.num_sats,
      self.enabled,
      self.mode_mask,
    ) = struct.unpack('<dddfffffffBBB', d)

  def to_binary(self):
    return struct.pack('<dddfffffffBBB',
      self.base_ecef_x,
      self.base_ecef_y,
      self.base_ecef_z,
      self.speed,
      self.radius,
      self.pos_variance,
      self.speed_variance,
      self.tracking_cn0_variance,
      self.pseudorange_variance,
      self.carrier_phase_variance,
      self.num_sats,
      self.enabled,
      self.mode_mask,
    )
  
  def to_list(self):
    l = [];
    l.append(['Simulation Enabled', self.enabled])
    l.append(['Simulator Mode Mask', self.mode_mask])
    l.append(['Speed (m/s)', self.speed])
    l.append(['Circle Radius (m)', self.radius])
    l.append(['Position Variance (m^2)', self.pos_variance])
    l.append(['Speed Variance (m^2)', self.speed_variance])
    l.append(['Tracking CN0 Variance (m^2)', self.tracking_cn0_variance])
    l.append(['Pseudorange Variance (m^2)', self.pseudorange_variance])
    l.append(['Carrier phase Variance (lambda^2)', self.carrier_phase_variance])
    l.append(['Centerpoint Coordinate X (ECEF)', self.base_ecef_x])
    l.append(['Centerpoint Coordinate Y (ECEF)', self.base_ecef_y])
    l.append(['Centerpoint Coordinate Z (ECEF)', self.base_ecef_z])
    l.append(['Simulated Number of Sats', self.num_sats])
    return l

  def from_list(self, l):
    self.enabled = int(l[0][1]) % 256
    self.mode_mask = int(l[1][1]) % 256
    self.speed = float(l[2][1])
    self.radius = float(l[3][1])
    self.pos_variance = float(l[4][1])
    self.speed_variance = float(l[5][1])
    self.tracking_cn0_variance = float(l[6][1])
    self.pseudorange_variance = float(l[7][1])
    self.carrier_phase_variance = float(l[8][1])
    self.base_ecef_x = float(l[9][1])
    self.base_ecef_y = float(l[10][1])
    self.base_ecef_z = float(l[11][1])
    self.num_sats = int(l[12][1])


class SimpleAdapter(TabularAdapter):
  columns = [('Setting', 0), ('Value',  1)]

class SimulatorView(HasTraits):

  simulator_enable_button = Button(label='Enable hardware\'s simulation mode')
  simulator_disable_button = Button(label='Disable hardware\'s simulation mode')
  simulator_get_settings_button = Button(label='Get simulation settings from Piksi')
  simulator_set_settings_button = Button(label='Send simulation settings to Piksi')

  settings_table = List()

  traits_view = View(
    HSplit(
      Item('settings_table', editor = TabularEditor(adapter=SimpleAdapter()), show_label=False, width=0.6),
      VGroup(
        Item('simulator_enable_button', show_label=False),
        Item('simulator_disable_button', show_label=False),
        Item('simulator_get_settings_button', show_label=False),
        Item('simulator_set_settings_button', show_label=False)
      ),
    )
  )

  ##Simulator buttons
  def _simulator_enable_button_fired(self):
    print "Requesting piksi to enter simulation mode"
    data = struct.pack("<B", 1)
    self.link.send_message(ids.SIMULATION_ENABLED, data)

  def _simulator_disable_button_fired(self):
    print "Requesting piksi to exit simulation mode"
    data = struct.pack("<B", 0)
    self.link.send_message(ids.SIMULATION_ENABLED, data)

  def _simulator_get_settings_button_fired(self):
    print "Requesting simulation settings..."
    self.link.send_message(ids.SIMULATION_SETTINGS, '');

  def _simulator_set_settings_button_fired(self):
    print "Sending new simulation settings..."
    simulation_settings = SimulationSettings(l=self.settings_table)
    self.link.send_message(ids.SIMULATION_SETTINGS, simulation_settings.to_binary())

  ##Callbacks for receiving messages

  def simulation_settings_message_callback(self, data):
    simulation_settings = SimulationSettings(d=data)
    self.settings_table = simulation_settings.to_list()

  def simulation_enabled_message_callback(self, data):
    self.link.send_message(ids.SIMULATION_SETTINGS, '');
  
  def __init__(self, link):
    super(SimulatorView, self).__init__()

    self.link = link
    self.link.add_callback(ids.SIMULATION_SETTINGS, self.simulation_settings_message_callback)
    self.link.add_callback(ids.SIMULATION_ENABLED, self.simulation_enabled_message_callback)

    #On startup, we request the current simulation mode and settings
    self.link.send_message(ids.SIMULATION_ENABLED, '');

    self.python_console_cmds = {
      'sim': self
    }


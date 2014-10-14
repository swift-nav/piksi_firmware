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


class SimulatorView(HasTraits):

  simulator_enable_button = Button(label='Enable hardware\'s simulation mode')
  simulator_disable_button = Button(label='Disable hardware\'s simulation mode')

  settings_table = List()

  traits_view = View(
    HGroup(
      Item('simulator_enable_button', show_label=False),
      Item('simulator_disable_button', show_label=False),
    ),
  )

  # Simulator buttons.
  def _simulator_enable_button_fired(self):
    print "Requesting piksi to enter simulation mode"
    data = struct.pack("<B", 1)
    self.link.send_message(ids.SIMULATION_ENABLED, data)

  def _simulator_disable_button_fired(self):
    print "Requesting piksi to exit simulation mode"
    data = struct.pack("<B", 0)
    self.link.send_message(ids.SIMULATION_ENABLED, data)

  # Callbacks for receiving messages.

  def simulation_enabled_message_callback(self, data):
    self.link.send_message(ids.SIMULATION_SETTINGS, '');

  def __init__(self, link):
    super(SimulatorView, self).__init__()

    self.link = link
    self.link.add_callback(ids.SIMULATION_ENABLED, self.simulation_enabled_message_callback)

    # On startup, we request the current simulation mode and settings.
    self.link.send_message(ids.SIMULATION_ENABLED, '');

    self.python_console_cmds = {
      'sim': self
    }


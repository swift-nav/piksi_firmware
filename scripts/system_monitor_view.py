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

class SimpleAdapter(TabularAdapter):
    columns = [('Name', 0), ('CPU %',  1)]

class ThreadState:
  def from_binary(self, data):
    state = struct.unpack('<20sH', data)
    self.name = state[0].rstrip('\0')
    if self.name == '':
      self.name = '(no name)'
    self.cpu = 100 * state[1] / 1000.


class SystemMonitorView(HasTraits):
  python_console_cmds = Dict()

  _threads_table_list = List()
  threads = List()

  traits_view = View(
    VGroup(
      Item('_threads_table_list', style = 'readonly', editor = TabularEditor(adapter=SimpleAdapter()), show_label=False),
    )
  )

  def update_threads(self):
    self._threads_table_list = [(thread_name, state.cpu) for thread_name, state in sorted(self.threads, key=lambda x: x[1].cpu, reverse=True)]

  def heartbeat_callback(self, data):
    self.update_threads()
    self.threads = []

  def thread_state_callback(self, data):
    th = ThreadState()
    th.from_binary(data)
    self.threads.append((th.name, th))

  def __init__(self, link):
    super(SystemMonitorView, self).__init__()

    self.link = link
    self.link.add_callback(ids.HEARTBEAT, self.heartbeat_callback)
    self.link.add_callback(ids.THREAD_STATE, self.thread_state_callback)

    self.python_console_cmds = {
      'mon': self
    }


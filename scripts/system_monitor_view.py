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

import os, sys
lib_path = os.path.abspath('../libswiftnav/sbp_generate')
sys.path.append(lib_path)
import sbp_messages

class SimpleAdapter(TabularAdapter):
    columns = [('Thread Name', 0), ('CPU %',  1), ('Stack Free',  2)]

class ThreadState:
  def from_binary(self, data):
    state = struct.unpack('<20sHI', data)
    self.name = state[0].rstrip('\0')
    if self.name == '':
      self.name = '(no name)'
    self.cpu = 100 * state[1] / 1000.
    self.stack_free = state[2]


class SystemMonitorView(HasTraits):
  python_console_cmds = Dict()

  _threads_table_list = List()
  threads = List()

  uart_a_crc_error_count = Int(0)
  uart_a_rx = Float(0)
  uart_a_tx = Float(0)

  uart_b_crc_error_count = Int(0)
  uart_b_rx = Float(0)
  uart_b_tx = Float(0)

  ftdi_crc_error_count = Int(0)
  ftdi_rx = Float(0)
  ftdi_tx = Float(0)

  traits_view = View(
    HSplit(
      Item(
        '_threads_table_list', style = 'readonly',
        editor = TabularEditor(adapter=SimpleAdapter()),
        show_label=False, width=0.85,
      ),
      VGroup(
        VGroup(
          Item('uart_a_crc_error_count', label='CRC Errors', style='readonly'),
          Item('uart_a_tx', label='TX Buffer %',
               style='readonly', format_str='%.1f'),
          Item('uart_a_rx', label='RX Buffer %',
               style='readonly', format_str='%.1f'),
          label='UART A', show_border=True,
        ),
        VGroup(
          Item('uart_b_crc_error_count', label='CRC Errors', style='readonly'),
          Item('uart_b_tx', label='TX Buffer %',
               style='readonly', format_str='%.1f'),
          Item('uart_b_rx', label='RX Buffer %',
               style='readonly', format_str='%.1f'),
          label='UART B', show_border=True,
        ),
        VGroup(
          Item('ftdi_crc_error_count', label='CRC Errors', style='readonly'),
          Item('ftdi_tx', label='TX Buffer %',
               style='readonly', format_str='%.1f'),
          Item('ftdi_rx', label='RX Buffer %',
               style='readonly', format_str='%.1f'),
          label='USB UART', show_border=True,
        ),
      ),
    )
  )

  def update_threads(self):
    self._threads_table_list = [(thread_name, state.cpu, state.stack_free) for thread_name, state in sorted(self.threads, key=lambda x: x[1].cpu, reverse=True)]

  def heartbeat_callback(self, data):
    self.update_threads()
    self.threads = []

  def thread_state_callback(self, data):
    th = ThreadState()
    th.from_binary(data)
    self.threads.append((th.name, th))

  def uart_state_callback(self, data):
    state = struct.unpack('<HBBHBBHBB', data)
    self.uart_a_crc_error_count = state[0]
    self.uart_a_tx, self.uart_a_rx = map(lambda x: 100.0 * x / 255.0, state[1:3])
    self.uart_b_crc_error_count = state[3]
    self.uart_b_tx, self.uart_b_rx = map(lambda x: 100.0 * x / 255.0, state[4:6])
    self.ftdi_crc_error_count = state[6]
    self.ftdi_tx, self.ftdi_rx = map(lambda x: 100.0 * x / 255.0, state[7:9])

  def __init__(self, link):
    super(SystemMonitorView, self).__init__()

    self.link = link
    self.link.add_callback(sbp_messages.SBP_HEARTBEAT, self.heartbeat_callback)
    self.link.add_callback(ids.THREAD_STATE, self.thread_state_callback)
    self.link.add_callback(ids.UART_STATE, self.uart_state_callback)

    self.python_console_cmds = {
      'mon': self
    }


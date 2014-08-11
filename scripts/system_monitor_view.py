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

import sbp_piksi as sbp_messages

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
  uart_a_rx_buffer = Float(0)
  uart_a_tx_buffer = Float(0)
  uart_a_tx_KBps = Float(0)
  uart_a_rx_KBps = Float(0)

  uart_b_crc_error_count = Int(0)
  uart_b_rx_buffer = Float(0)
  uart_b_tx_buffer = Float(0)
  uart_b_tx_KBps = Float(0)
  uart_b_rx_KBps = Float(0)


  ftdi_crc_error_count = Int(0)
  ftdi_rx_buffer = Float(0)
  ftdi_tx_buffer = Float(0)
  ftdi_tx_KBps = Float(0)
  ftdi_rx_KBps = Float(0)

  msg_obs_avg_latency_ms    = Int(0)
  msg_obs_min_latency_ms    = Int(0)
  msg_obs_max_latency_ms    = Int(0)
  msg_obs_window_latency_ms = Int(0)

  traits_view = View(
    HSplit(
      Item(
        '_threads_table_list', style = 'readonly',
        editor = TabularEditor(adapter=SimpleAdapter()),
        show_label=False, width=0.85,
      ),
      VGroup(
        VGroup(
          Item('msg_obs_window_latency_ms', label='Obs Latency',
            style='readonly', format_str='%dms'),
          Item('msg_obs_avg_latency_ms', label='Obs Latency (Avg ms)',
            style='readonly', format_str='%dms'),
          Item('msg_obs_min_latency_ms', label='Obs Latency (Min ms)',
            style='readonly', format_str='%dms'),
          Item('msg_obs_max_latency_ms', label='Obs Latency (Max ms)',
            style='readonly', format_str='%dms'),
          label='Connection Monitor', show_border=True,
        ),
        VGroup(
          Item('uart_a_crc_error_count', label='CRC Errors',
            style='readonly'),
          Item('uart_a_tx_buffer', label='TX Buffer %',
               style='readonly', format_str='%.1f'),
          Item('uart_a_rx_buffer', label='RX Buffer %',
               style='readonly', format_str='%.1f'),
          Item('uart_a_tx_KBps', label='TX KBytes/s',
               style='readonly', format_str='%.2f'),
          Item('uart_a_rx_KBps', label='RX KBytes/s',
               style='readonly', format_str='%.2f'),
          label='UART A', show_border=True,
        ),
        VGroup(
          Item('uart_b_crc_error_count', label='CRC Errors',
            style='readonly'),
          Item('uart_b_tx_buffer', label='TX Buffer %',
               style='readonly', format_str='%.1f'),
          Item('uart_b_rx_buffer', label='RX Buffer %',
               style='readonly', format_str='%.1f'),
          Item('uart_b_tx_KBps', label='TX KBytes/s',
               style='readonly', format_str='%.2f'),
          Item('uart_b_rx_KBps', label='RX KBytes/s',
               style='readonly', format_str='%.2f'),
          label='UART B', show_border=True,
        ),
        VGroup(
          Item('ftdi_crc_error_count', label='CRC Errors',
            style='readonly'),
          Item('ftdi_tx_buffer', label='TX Buffer %',
               style='readonly', format_str='%.1f'),
          Item('ftdi_rx_buffer', label='RX Buffer %',
               style='readonly', format_str='%.1f'),
          Item('ftdi_tx_KBps', label='TX KBytes/s',
               style='readonly', format_str='%.2f'),
          Item('ftdi_rx_KBps', label='RX KBytes/s',
               style='readonly', format_str='%.2f'),
          label='USB UART', show_border=True,
        ),
      ),
    )
  )

  def update_threads(self):
    self._threads_table_list = [(thread_name, state.cpu, state.stack_free)
      for thread_name, state in sorted(
        self.threads, key=lambda x: x[1].cpu, reverse=True)]

  def heartbeat_callback(self, data):
    self.update_threads()
    self.threads = []

  def thread_state_callback(self, data):
    th = ThreadState()
    th.from_binary(data)
    self.threads.append((th.name, th))

  def uart_state_callback(self, data):
    state = struct.unpack('<ffHBBffHBBffHBBiiii', data)
    self.uart_a_tx_KBps, self.uart_a_rx_KBps = state[0:2]
    self.uart_a_crc_error_count = state[2]
    self.uart_a_tx_buffer, self.uart_a_rx_buffer = map(
      lambda x: 100.0 * x / 255.0, state[3:5])

    self.uart_b_tx_KBps, self.uart_b_rx_KBps = state[5:7]
    self.uart_b_crc_error_count = state[7]
    self.uart_b_tx_buffer, self.uart_b_rx_buffer = map(
      lambda x: 100.0 * x / 255.0, state[8:10])

    self.ftdi_tx_KBps, self.ftdi_rx_KBps = state[10:12]
    self.ftdi_crc_error_count = state[12]
    self.ftdi_tx_buffer, self.ftdi_rx_buffer = map(
      lambda x: 100.0 * x / 255.0, state[13:15])

    self.msg_obs_avg_latency_ms = state[15]
    self.msg_obs_min_latency_ms = state[16]
    self.msg_obs_max_latency_ms = state[17]
    self.msg_obs_window_latency_ms = state[18]


  def __init__(self, link):
    super(SystemMonitorView, self).__init__()

    self.link = link
    self.link.add_callback(sbp_messages.SBP_HEARTBEAT,
      self.heartbeat_callback)
    self.link.add_callback(sbp_messages.THREAD_STATE,
      self.thread_state_callback)
    self.link.add_callback(sbp_messages.UART_STATE,
      self.uart_state_callback)


    self.python_console_cmds = {
      'mon': self
    }


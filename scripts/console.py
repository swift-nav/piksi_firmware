#!/usr/bin/env python

import logging
logging.basicConfig()

# Fix default font issue on Linux
import os
from enthought.kiva.fonttools.font_manager import fontManager, FontProperties
if os.name == "posix":
  font = FontProperties()
  font.set_name("DejaVu Sans")
  fontManager.defaultFont = fontManager.findfont(font)

from enthought.traits.api import Str, Instance, Dict, HasTraits, Int
from enthought.traits.ui.api import Item, ShellEditor, View, VSplit, HSplit, Tabbed, InstanceEditor

import struct

import serial_link
from output_stream import OutputStream
from tracking_view import TrackingView
from almanac_view import AlmanacView

class SwiftConsole(HasTraits):
  link = Instance(serial_link.SerialLink)
  console_output = Instance(OutputStream)
  python_console_env = Dict
  a = Int
  b = Int
  tracking_view = Instance(TrackingView)
  almanac_view = Instance(AlmanacView)

  view = View(
    VSplit(
      Tabbed(
        Item('tracking_view', style='custom', show_label=False),
        Item('almanac_view', style='custom', show_label=False),
      ),
      HSplit(
        Item('python_console_env', editor=ShellEditor()),
        Item('console_output', style='custom', editor=InstanceEditor()),
        Item('tracking_view', style='custom', editor=InstanceEditor(view='snr_bar_view')),
        show_labels=False
      )
    ),
    kind='live'
  )

  def print_message_callback(self, data):
    self.console_output.write(data.encode('ascii', 'ignore'))

  def __init__(self):
    self.console_output = OutputStream()

    self.link = serial_link.SerialLink(serial_link.DEFAULT_PORT, serial_link.DEFAULT_BAUD)
    self.link.add_callback(serial_link.MSG_PRINT, self.print_message_callback)

    self.tracking_view = TrackingView(self.link)
    self.almanac_view = AlmanacView(self.link)

    self.python_console_env = {
        'send_message': self.link.send_message
    }
    self.python_console_env.update(self.tracking_view.python_console_cmds)
    self.python_console_env.update(self.almanac_view.python_console_cmds)

  def stop(self):
    self.link.close()

console = SwiftConsole()

console.configure_traits()
console.stop()

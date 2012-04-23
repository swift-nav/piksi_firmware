#!/usr/bin/env python

import serial_link

import argparse
parser = argparse.ArgumentParser(description='Swift Nav Console.')
parser.add_argument('-p', '--port',
                   default=[serial_link.DEFAULT_PORT], nargs=1,
                   help='specify the serial port to use.')
args = parser.parse_args()
serial_port = args.port[0]

import logging
logging.basicConfig()

# Fix default font issue on Linux
import os
from enthought.kiva.fonttools.font_manager import fontManager, FontProperties
if os.name == "posix":
  font = FontProperties()
  font.set_name("Arial")
  fontManager.defaultFont = fontManager.findfont(font)

from enthought.traits.api import Str, Instance, Dict, HasTraits, Int
from enthought.traits.ui.api import Item, ShellEditor, View, VSplit, HSplit, Tabbed, InstanceEditor

import struct

from output_stream import OutputStream
from tracking_view import TrackingView
from almanac_view import AlmanacView
from solution_view import SolutionView
import flash

class SwiftConsole(HasTraits):
  link = Instance(serial_link.SerialLink)
  console_output = Instance(OutputStream)
  python_console_env = Dict
  a = Int
  b = Int
  tracking_view = Instance(TrackingView)
  almanac_view = Instance(AlmanacView)
  solution_view = Instance(SolutionView)

  view = View(
    VSplit(
      Tabbed(
        Item('solution_view', style='custom', show_label=False),
        Item('tracking_view', style='custom', show_label=False),
        Item('tracking_view', style='custom', show_label=False, editor=InstanceEditor(view='snr_line_view')),
        Item('solution_view', style='custom', show_label=False, editor=InstanceEditor(view='prs_view')),
        Item('almanac_view', style='custom', show_label=False),
      ),
      HSplit(
        Item('python_console_env', editor=ShellEditor()),
        Item('console_output', style='custom', editor=InstanceEditor()),
        Item('tracking_view', style='custom', editor=InstanceEditor(view='snr_bar_view')),
        show_labels=False
      )
    ),
    resizable = True,
    width = 1000,
    height = 600
  )

  def print_message_callback(self, data):
    try:
      self.console_output.write(data.encode('ascii', 'ignore'))
    except UnicodeDecodeError:
      print "Oh crap!"

  def __init__(self, port=serial_link.DEFAULT_PORT):
    self.console_output = OutputStream()

    self.link = serial_link.SerialLink(port)
    self.link.add_callback(serial_link.MSG_PRINT, self.print_message_callback)

    self.tracking_view = TrackingView(self.link)
    self.almanac_view = AlmanacView(self.link)
    self.solution_view = SolutionView(self.link)

    self.flash = flash.Flash(self.link)
    self.flash.start()
    self.python_console_env = {
        'send_message': self.link.send_message,
        'link': self.link,
        'flash': self.flash
    }
    self.python_console_env.update(self.tracking_view.python_console_cmds)
    self.python_console_env.update(self.almanac_view.python_console_cmds)
    self.python_console_env.update(self.solution_view.python_console_cmds)

  def stop(self):
    self.flash.stop()
    self.link.close()

console = SwiftConsole(serial_port)

console.configure_traits()
console.stop()

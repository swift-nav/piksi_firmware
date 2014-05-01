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

import pyface.ui.qt4.resource_manager
#import pyface.ui.qt4.python_shell
#from pygments.lexers import PythonLexer
from pyface.image_resource import ImageResource

import serial_link
import sbp_piksi as ids

import argparse
parser = argparse.ArgumentParser(description='Swift Nav Console.')
parser.add_argument('-p', '--port', nargs=1, default=[None],
                   help='specify the serial port to use.')
parser.add_argument('-b', '--baud', nargs=1, default=[serial_link.DEFAULT_BAUD],
                   help='specify the baud rate to use.')
parser.add_argument("-v", "--verbose",
                  help="print extra debugging information.",
                  action="store_true")
parser.add_argument("-f", "--ftdi",
                  help="use pylibftdi instead of pyserial.",
                  action="store_true")
parser.add_argument('-t', '--toolkit', nargs=1, default=[None],
                   help="specify the TraitsUI toolkit to use, either 'wx' or 'qt4'.")
args = parser.parse_args()
serial_port = args.port[0]
baud = args.baud[0]

if args.toolkit[0] is not None:
  from traits.etsconfig.api import ETSConfig
  ETSConfig.toolkit = args.toolkit[0]

import logging
logging.basicConfig()

# Fix default font issue on Linux
import os
import sys
#from kiva.fonttools.font_manager import fontManager, FontProperties
#if os.name == "posix":
  #font = FontProperties()
  #font.set_name("Arial")
  #fontManager.defaultFont = fontManager.findfont(font)

from traits.api import Str, Instance, Dict, HasTraits, Int, Button, List
from traitsui.api import Item, Label, View, VGroup, VSplit, HSplit, Tabbed, InstanceEditor, EnumEditor #, ShellEditor

import struct

if getattr(sys, 'frozen', False):
    # we are running in a |PyInstaller| bundle
    basedir = sys._MEIPASS
    os.chdir(basedir)
else:
    # we are running in a normal Python environment
    basedir = os.path.dirname(__file__)
icon = ImageResource('icon',
         search_path=['images', os.path.join(basedir, 'images')])

from output_stream import OutputStream
from tracking_view import TrackingView
from almanac_view import AlmanacView
from solution_view import SolutionView
from baseline_view import BaselineView
from observation_view import ObservationView
from system_monitor_view import SystemMonitorView
from simulator_view import SimulatorView
from settings_view import SettingsView

class SwiftConsole(HasTraits):
  link = Instance(serial_link.SerialLink)
  console_output = Instance(OutputStream)
  python_console_env = Dict
  a = Int
  b = Int
  tracking_view = Instance(TrackingView)
  almanac_view = Instance(AlmanacView)
  solution_view = Instance(SolutionView)
  baseline_view = Instance(BaselineView)
  observation_view = Instance(ObservationView)
  observation_view_base = Instance(ObservationView)
  system_monitor_view = Instance(SystemMonitorView)
  simulator_view = Instance(SimulatorView)
  settings_view = Instance(SettingsView)

  view = View(
    VSplit(
      Tabbed(
        Item('tracking_view', style='custom', label='Tracking'),
        Item('almanac_view', style='custom', label='Almanac'),
        Item('solution_view', style='custom', label='Solution'),
        Item('baseline_view', style='custom', label='Baseline'),
        VSplit(
          Item('observation_view', style='custom', show_label=False),
          Item('observation_view_base', style='custom', show_label=False),
          label='Observations',
        ),
        Item('simulator_view', style='custom', label='Simulator'),
        Item('settings_view', style='custom', label='Settings'),
        Item('system_monitor_view', style='custom', label='System Monitor'),
        #Item(
          #'python_console_env', style='custom',
          #label='Python Console', editor=ShellEditor()
        #),
        show_labels=False
      ),
      Item(
        'console_output',
        style='custom',
        editor=InstanceEditor(),
        height=0.3,
        show_label=False,
      ),
    ),
    icon = icon,
    resizable = True,
    width = 1000,
    height = 800,
    title = 'Piksi console'
  )

  def print_message_callback(self, data):
    try:
      self.console_output.write(data.encode('ascii', 'ignore'))
    except UnicodeDecodeError:
      print "Oh crap!"

  def debug_var_callback(self, data):
    x = struct.unpack('<d', data[:8])[0]
    name = data[8:]
    print "VAR: %s = %d" % (name, x)

  def __init__(self, *args, **kwargs):
    self.console_output = OutputStream()
    sys.stdout = self.console_output
    sys.stderr = self.console_output

    try:
      self.link = serial_link.SerialLink(*args, **kwargs)
      self.link.add_callback(ids.PRINT, self.print_message_callback)

      self.link.add_callback(ids.DEBUG_VAR, self.debug_var_callback)

      self.tracking_view = TrackingView(self.link)
      self.almanac_view = AlmanacView(self.link)
      self.solution_view = SolutionView(self.link)
      self.baseline_view = BaselineView(self.link)
      self.observation_view = ObservationView(self.link, name='Rover', relay=False)
      self.observation_view_base = ObservationView(self.link, name='Base', relay=True)
      self.system_monitor_view = SystemMonitorView(self.link)
      self.simulator_view = SimulatorView(self.link)
      self.settings_view = SettingsView(self.link)

      self.python_console_env = {
          'send_message': self.link.send_message,
          'link': self.link,
      }
      self.python_console_env.update(self.tracking_view.python_console_cmds)
      self.python_console_env.update(self.almanac_view.python_console_cmds)
      self.python_console_env.update(self.solution_view.python_console_cmds)
      self.python_console_env.update(self.baseline_view.python_console_cmds)
      self.python_console_env.update(self.observation_view.python_console_cmds)
      self.python_console_env.update(self.system_monitor_view.python_console_cmds)
    except:
      import traceback
      traceback.print_exc()

  def stop(self):
    self.link.close()

class PortChooser(HasTraits):
  ports = List()
  port = Str(None)
  traits_view = View(
    VGroup(
      Label('Select Piksi device:'),
      Item('port', editor=EnumEditor(name='ports'), show_label=False),
    ),
    buttons = ['OK', 'Cancel'],
    icon = icon,
    title = 'Select serial device'
  )

  def __init__(self):
    self.ports = [p for p, _, _ in serial_link.list_ports()]

if serial_port is None:
  port_chooser = PortChooser()
  port_chooser.configure_traits()
  serial_port = port_chooser.port
  if serial_port is None:
    print "No serial device selected!"
    import sys
    sys.exit(1)
  else:
    print "Using serial device '%s'" % serial_port

console = SwiftConsole(serial_port, baud, use_ftdi=args.ftdi,
                       print_unhandled=args.verbose)

console.configure_traits()
console.stop()

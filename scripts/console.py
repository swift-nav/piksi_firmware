#!/usr/bin/env python

from enthought.traits.api import Str, Instance, Dict, HasTraits, Int
from enthought.traits.ui.api import Item, ShellEditor, View, VSplit, HSplit, Tabbed, InstanceEditor

import seriallink
from output_stream import OutputStream

class SwiftConsole(HasTraits):
  link = Instance(seriallink.SerialLink)
  console_output = Instance(OutputStream)
  python_console_env = Dict
  a = Int
  b = Int

  view = View(
    VSplit(
      Tabbed(
        Item('a'),
        Item('b')
      ),
      HSplit(
        Item('python_console_env', editor=ShellEditor()),
        Item('console_output', style='custom', editor=InstanceEditor()),
        show_labels=False
      )
    )
  )

  def print_message_callback(self, data):
    self.console_output.write(data.encode('ascii', 'ignore'))

  def __init__(self):
    self.console_output = OutputStream()

    self.link = seriallink.SerialLink(seriallink.DEFAULT_PORT, seriallink.DEFAULT_BAUD)
    self.link.add_callback(seriallink.MSG_PRINT, self.print_message_callback)

    self.python_console_env = {
        'send_message': self.link.send_message
    }

  def stop(self):
    self.link.close()

console = SwiftConsole()

console.configure_traits()
console.stop()

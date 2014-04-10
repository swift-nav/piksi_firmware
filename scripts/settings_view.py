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
  columns = [('Setting', 0), ('Value',  1)]

class SettingsView(HasTraits):

  settings_read_button = Button(label='Read settings')
  settings_save_button = Button(label='Save settings to config file')

  settings_table = List()

  traits_view = View(
    HSplit(
      Item('settings_table', editor = TabularEditor(adapter=SimpleAdapter()), show_label=False, width=0.6),
      VGroup(
        Item('settings_read_button', show_label=False),
        Item('settings_save_button', show_label=False)
      ),
    )
  )

  def _settings_table_changed(self, old, new):
    print old, new

  ##Simulator buttons
  def _settings_read_button_fired(self):
    print "Requesting settings from piksi"
    self.settings = {}
    self.discover_state = True
    self.link.send_message(ids.SETTINGS, "")

  def _settings_save_button_fired(self):
    print "Saving settings to filesystem"
    self.link.send_message(ids.SETTINGS_SAVE, "")

  ##Callbacks for receiving messages

  def settings_read_callback(self, data):
    print repr(data)
    section, setting, value, _ = data.split('\0')
    key = section + '.' + setting
    if self.discover_state:
      if self.settings.has_key(key):
        self.discover_state = False
        print self.settings
        self.settings_table = []
        keys = self.settings.keys()
        keys.sort()
        for k in keys:
          self.settings_table.append([k, self.settings[k]])
      else:
        print "Found setting: %s = %s" % (key, value)
        self.link.send_message(ids.SETTINGS, "")
    else:
      print "Setting updated: %s = %s" % (key, value)
    self.settings[section + '.' + setting] = value

  def __init__(self, link):
    super(SettingsView, self).__init__()

    self.settings = {}
    self.link = link
    self.discover_state = False
    self.link.add_callback(ids.SETTINGS, self.settings_read_callback)

    self.python_console_cmds = {
      'sim': self
    }


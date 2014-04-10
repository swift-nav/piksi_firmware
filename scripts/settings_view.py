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

from traits.api import Instance, Dict, HasTraits, Array, Float, on_trait_change, List, Int, Button, Bool, Str, Color, Constant, Font, Undefined
from traitsui.api import Item, View, HGroup, VGroup, ArrayEditor, HSplit, TabularEditor
from traitsui.tabular_adapter import TabularAdapter

import struct
import math
import os
import numpy as np
import datetime

import sbp_piksi as ids



class SettingBase(HasTraits):
  name = Str()
  value = Str(Undefined)

class Setting(SettingBase):
  section = Str()

  def __init__(self, name, section, value, link):
    self.name = name
    self.section = section
    self.value = value
    self.link = link

  def _value_changed(self, name, old, new):
    if (old != new and
        old is not Undefined and
        new is not Undefined):
      print old, new
      if type(self.value) == unicode:
        self.value = self.value.encode('ascii', 'replace')
      print (self.section, self.name, self.value)
      print repr('%s\0%s\0%s' % (self.section, self.name, self.value))
      self.link.send_message(ids.SETTINGS, '%s\0%s\0%s' % (self.section, self.name, self.value))

class SectionHeading(SettingBase):
  value = Constant('')

  def __init__(self, name):
    self.name = name

class SimpleAdapter(TabularAdapter):
  columns = [('Name', 'name'), ('Value',  'value')]
  SectionHeading_bg_color = Color(0xE0E0E0)
  SectionHeading_font = Font('bold')
  SectionHeading_can_edit = Bool(False)
  #Setting_name_can_edit = Bool(False)
  #Setting_value_can_edit = Bool(True)

class SettingsView(HasTraits):

  settings_read_button = Button(label='Read settings')
  settings_save_button = Button(label='Save settings to config file')

  settings_list = List(SettingBase)

  traits_view = View(
    HSplit(
      Item('settings_list', editor = TabularEditor(adapter=SimpleAdapter(), editable_labels=False), show_label=False, width=0.6),
      VGroup(
        Item('settings_read_button', show_label=False),
        Item('settings_save_button', show_label=False)
      ),
    )
  )

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
    section, setting, value, _ = data.split('\0')

    if self.discover_state:
      if self.settings.has_key(section) and self.settings[section].has_key(setting):
        self.discover_state = False
        print self.settings
        self.settings_list = []

        sections = sorted(self.settings.keys())

        for sec in sections:
          self.settings_list.append(SectionHeading(sec))
          for setting in sorted(self.settings[sec].keys()):
            self.settings_list.append(self.settings[sec][setting])

      else:
        print "Found setting: %s.%s = %s" % (section, setting, value)
        if not self.settings.has_key(section):
          self.settings[section] = {}
        self.settings[section][setting] = Setting(setting, section, value, link=self.link)
        self.link.send_message(ids.SETTINGS, "")

    else:
      print "Setting updated: %s.%s = %s" % (section, setting, value)
      # Hack to prevent an infinite loop of setting settings
      self.settings[section][setting].value = Undefined
      self.settings[section][setting].value = value

  def __init__(self, link):
    super(SettingsView, self).__init__()

    self.settings = {}
    self.link = link
    self.discover_state = False
    self.link.add_callback(ids.SETTINGS, self.settings_read_callback)

    self.python_console_cmds = {
      'settings': self
    }


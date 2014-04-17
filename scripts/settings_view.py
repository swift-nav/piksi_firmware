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

from traits.api import Instance, Dict, HasTraits, Array, Float, on_trait_change, List, Int, Button, Bool, Str, Color, Constant, Font, Undefined, Property, Any, Enum
from traitsui.api import Item, View, HGroup, VGroup, ArrayEditor, HSplit, TabularEditor, TextEditor, EnumEditor
from traitsui.tabular_adapter import TabularAdapter
from enable.savage.trait_defs.ui.svg_button import SVGButton

import struct
import math
import os
import numpy as np
import datetime

import sbp_piksi as ids

def u16_to_str(i):
  return chr(i & 0xff) + chr(i >> 8)

class SettingBase(HasTraits):
  name = Str()
  description = Str()
  value = Str(Undefined)
  ordering = Float(0)

  traits_view = View()

class Setting(SettingBase):
  full_name = Str()
  section = Str()

  traits_view = View(
    VGroup(
      Item('full_name', label='Name', style='readonly'),
      Item('value', editor=TextEditor(auto_set=False, enter_set=True)),
      Item('description', style='readonly'),
      show_border=True,
      label='Setting',
    ),
  )

  def __init__(self, name, section, value, link, ordering):
    self.name = name
    self.section = section
    self.full_name = "%s.%s" % (section, name)
    self.value = value
    self.link = link
    self.ordering = ordering

  def _value_changed(self, name, old, new):
    if (old != new and
        old is not Undefined and
        new is not Undefined):
      if type(self.value) == unicode:
        self.value = self.value.encode('ascii', 'replace')
      self.link.send_message(ids.SETTINGS,
          '%s\0%s\0%s\0' % (self.section, self.name, self.value))

class EnumSetting(Setting):
  values = List()
  traits_view = View(
    VGroup(
      Item('full_name', label='Name', style='readonly'),
      Item('value', editor=EnumEditor(name='values')),
      Item('description', style='readonly'),
      show_border=True,
      label='Setting',
    ),
  )

  def __init__(self, name, section, value, link, ordering, values):
    self.values = values
    Setting.__init__(self, name, section, value, link, ordering)

class SectionHeading(SettingBase):
  value = Constant('')

  def __init__(self, name):
    self.name = name

class SimpleAdapter(TabularAdapter):
  columns = [('Name', 'name'), ('Value',  'value')]
  font = Font('12')
  can_edit = Bool(False)
  SectionHeading_bg_color = Color(0xE0E0E0)
  SectionHeading_font = Font('14 bold')
  SectionHeading_name_text = Property
  Setting_name_text = Property
  name_width = Float(0.5)
  value_width = Float(0.9)

  def _get_SectionHeading_name_text(self):
    return self.item.name.replace('_', ' ')

  def _get_Setting_name_text(self):
    return self.item.name.replace('_', ' ')

class SettingsView(HasTraits):

  settings_read_button = SVGButton(
    label='Reload', tooltip='Reload settings from Piksi',
    filename=os.path.join(os.path.dirname(__file__), 'images', 'fontawesome', 'refresh.svg'),
    width=16, height=16
  )

  settings_save_button = SVGButton(
    label='Save to Flash', tooltip='Save settings to Flash',
    filename=os.path.join(os.path.dirname(__file__), 'images', 'fontawesome', 'download.svg'),
    width=16, height=16
  )

  settings_list = List(SettingBase)
  selected_setting = Instance(SettingBase)

  traits_view = View(
    HSplit(
      Item('settings_list',
        editor = TabularEditor(
          adapter=SimpleAdapter(),
          editable_labels=False,
          auto_update=True,
          selected='selected_setting'
        ),
        show_label=False,
      ),
      VGroup(
        HGroup(
          Item('settings_read_button', show_label=False),
          Item('settings_save_button', show_label=False),
        ),
        Item('selected_setting', style='custom', show_label=False),
      ),
    )
  )

  ##Simulator buttons
  def _settings_read_button_fired(self):
    self.settings = {}
    self.enumindex = 0
    self.ordering_counter = 0
    self.link.send_message(ids.SETTINGS_READ_BY_INDEX, u16_to_str(self.enumindex))

  def _settings_save_button_fired(self):
    self.link.send_message(ids.SETTINGS_SAVE, "")

  ##Callbacks for receiving messages

  def settings_read_by_index_callback(self, data):
    if not data:
      self.settings_list = []

      sections = sorted(self.settings.keys())

      for sec in sections:
        self.settings_list.append(SectionHeading(sec))
        for name, setting in sorted(self.settings[sec].iteritems(), key=lambda (n, s): s.ordering):
          self.settings_list.append(setting)
      return

    section, setting, value, format_type = data[2:].split('\0')[:4]
    self.ordering_counter += 1

    if format_type == '':
      format_type = None
    else:
      setting_type, setting_format = format_type.split(':')

    if not self.settings.has_key(section):
      self.settings[section] = {}

    if format_type is None:
      # Plain old setting, no format information
      self.settings[section][setting] = Setting(setting, section, value,
                                                link=self.link,
                                                ordering=self.ordering_counter
        )
    else:
      if setting_type == 'enum':
        enum_values = setting_format.split(',')
        self.settings[section][setting] = EnumSetting(setting, section, value,
                                                      link=self.link,
                                                      ordering=self.ordering_counter,
                                                      values=enum_values)
      else:
        # Unknown type, just treat is as a string
        self.settings[section][setting] = Setting(setting, section, value, link=self.link)

    self.enumindex += 1
    self.link.send_message(ids.SETTINGS_READ_BY_INDEX, u16_to_str(self.enumindex))

  def settings_read_callback(self, data):
    section, setting, value = data.split('\0')[:3]
    # Hack to prevent an infinite loop of setting settings
    self.settings[section][setting].value = Undefined
    self.settings[section][setting].value = value

  def piksi_startup_callback(self, data):
    self._settings_read_button_fired()

  def __init__(self, link):
    super(SettingsView, self).__init__()

    self.enumindex = 0
    self.settings = {}
    self.link = link
    self.link.add_callback(ids.SETTINGS, self.settings_read_callback)
    self.link.add_callback(ids.SBP_STARTUP, self.piksi_startup_callback)
    self.link.add_callback(ids.SETTINGS_READ_BY_INDEX,
        self.settings_read_by_index_callback)

    self.setting_detail = SettingBase()

    self._settings_read_button_fired()

    self.python_console_cmds = {
      'settings': self
    }


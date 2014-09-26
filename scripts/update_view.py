#!/usr/bin/env python
# Copyright (C) 2014 Swift Navigation Inc.
# Contact: Colin Beighley <colin@swift-nav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

# TODO: have update button blank out and not be clickable if firmware files
#       haven't been downloaded yet
# TODO: have Firmware Update tab blink if new firmware is available
# TODO: allow user to specify path to firmware file. save firmware file
#       locally. give user a button to control downloading the firmware.

from urllib2 import urlopen, URLError
from json import load as jsonload
from time import sleep
from intelhex import IntelHex
from new import instancemethod
from pkg_resources import parse_version

from threading import Thread

from traits.api import HasTraits, Event, String, Button
from traitsui.api import View, Handler, Action, Item, TextEditor, VGroup
from pyface.api import GUI

from version import VERSION as CONSOLE_VERSION
import bootload
import sbp_piksi as ids
import flash

import sys, os
from pyface.image_resource import ImageResource
if getattr(sys, 'frozen', False):
    # we are running in a |PyInstaller| bundle
    basedir = sys._MEIPASS
    os.chdir(basedir)
else:
    # we are running in a normal Python environment
    basedir = os.path.dirname(__file__)
icon = ImageResource('icon',
         search_path=['images', os.path.join(basedir, 'images')])

INDEX_URL = 'http://download.swift-nav.com/index.json'

update_button = Action(name = "Update", action = "set_execute_callback_true", \
                             show_label=False)
close_button = Action(name = "Close", action = "set_execute_callback_false", \
                             show_label=False)

# Handler methods that can be associated with buttons.
def set_execute_callback_true(self, info):
  info.object.execute_callback = True
  info.object.handler_executed = True

def set_execute_callback_false(self, info):
  info.object.execute_callback = False
  info.object.handler_executed = True

class UpdateHandler(Handler):

  # Register action handlers from passed list.
  def __init__(self, actions):
    super(UpdateHandler, self).__init__()
    for a in actions:
      # Add instancemethod to self for Action.action.
      handler = globals()[a.action]
      self.__dict__[a.action] = instancemethod(handler, self, UpdateHandler)

  # X button was pressed.
  def close(self, info, is_ok):
    info.object.handler_executed = True
    info.object.closed = True
    return True

  # Handles parent object's close Event being written to.
  def object_close_changed(self, info):
    info.object.closed = True
    info.ui.owner.close()

class UpdatePrompt(HasTraits):

  text = String
  close = Event

  def __init__(self, title, actions, update_callback=None):
    self.update_callback = update_callback

    self.handler_executed = False
    self.execute_callback = False
    self.closed = False
    self.close = 0

    self.view = View(
      Item(
        'text',
        style='readonly',
        editor=TextEditor(),
        show_label=False
      ),
      buttons=actions,
      title=title,
      handler=UpdateHandler(actions),
      icon = icon,
      resizable=True,
    )

  def run(self):

    GUI.invoke_later(self.edit_traits, self.view)
    while not self.handler_executed:
      sleep(0.1)

    if self.execute_callback:
      GUI.invoke_later(self.update_callback)

    if not self.closed:
      self.close = 1
    while not self.closed:
      sleep(0.1)

class UpdateView(HasTraits):

  piksi_stm_vers = String('Waiting for Piksi device info...')
  newest_stm_vers = String('Waiting for newest firmware info...')
  piksi_nap_vers = String('Waiting for Piksi device info...')
  newest_nap_vers = String('Waiting for newest firmware info...')
  update_firmware = Button(label='Update Piksi Firmware')

  traits_vew = View(
    VGroup(
      Item('piksi_stm_vers', label='Piksi STM Firmware Version'),
      Item('newest_stm_vers', label='Newest STM Firmware Version'),
      Item('piksi_nap_vers', label='Piksi NAP Firmware Version'),
      Item('newest_nap_vers', label='Newest NAP Firmware Version'),
      Item('update_firmware')
    )
  )

  def __init__(self, link, prompt=True):
    self.link = link
    self.settings = {}
    self.nap_ihx = None
    self.stm_ihx = None
    self.prompt = prompt
    self.updating = False
    self.python_console_cmds = {
      'update': self
    }


  def _update_firmware_fired(self):
    # Make sure we have firmware files.
    if not self.updating:
      if self.nap_ihx and self.stm_ihx:
        GUI.invoke_later(self.manage_firmware_updates)
      else:
        print "No firmware files have been downloaded yet!"

  # Instantiate an instance variable thread instead of inheriting Thread 
  # so start can be called multiple times.
  # Expectation is that OneClickUpdate.start is passed to SettingsView to
  # be called after settings are read out, which can happen multiple times.
  # This method is executed in GUI thread.
  def start(self):

    try:
      if self.thread.is_alive():
        return
    except AttributeError:
      pass

    self.thread = Thread(target=self.run)
    self.thread.start()

  def update_piksi_fw_vers(self):
    try:
      self.piksi_stm_vers = \
        self.settings['system_info']['firmware_version'].value
      self.piksi_nap_vers = \
        self.settings['system_info']['nap_version'].value
    except KeyError:
      print "\nError: Settings received from Piksi don't contain firmware version keys. Please contact Swift Navigation.\n\n"

  def point_to_settings(self, settings):
    self.settings = settings

  # Executed in it's own thread.
  def run(self):

    # Create prompt objects.
    fw_update_prompt = \
        UpdatePrompt(
                     title='Firmware Update',
                     actions=[close_button]
                    )

    console_outdated_prompt = \
        UpdatePrompt(
                     title="Piksi Console Outdated",
                     actions=[close_button],
                    )

    self.update_piksi_fw_vers()

    # Get index that contains file URLs and latest
    # version strings from Swift Nav's website.
    try:
      f = urlopen(INDEX_URL)
      index = jsonload(f)
      f.close()
    except URLError:
      print "\nError: Failed to download latest file index from Swift Navigation's website (%s). Please visit our website to check that you're running the latest Piksi firmware and Piksi console.\n\n" % INDEX_URL
      return

    # Make sure index contains all keys we are interested in.
    try:
      self.newest_stm_vers = index['piksi_v2.3.1']['stm_fw']['version']
      self.newest_nap_vers = index['piksi_v2.3.1']['nap_fw']['version']
      index['piksi_v2.3.1']['stm_fw']['url']
      index['piksi_v2.3.1']['nap_fw']['url']
      index['piksi_v2.3.1']['console']['version']
    except KeyError:
      print "\nError: Index downloaded from Swift Navigation's website (%s) doesn't contain all keys. Please contact Swift Navigation.\n\n" % INDEX_URL
      return

    # Assign text to UpdatePrompt's
    fw_update_prompt.text = \
        "New Piksi firmware available.\n" + \
        "Please use the Firmware Update tab to update.\n\n" + \
        "Newest STM Version :\n\t%s\n\n" % \
            index['piksi_v2.3.1']['stm_fw']['version'] + \
        "Newest SwiftNAP Version :\n\t%s\n\n" % \
            index['piksi_v2.3.1']['nap_fw']['version']

    console_outdated_prompt.text = \
        "Your Piksi Console is out of date and may be incompatible\n" + \
        "with current firmware. We highly recommend upgrading to\n" + \
        "ensure proper behavior.\n\n" + \
        "Please visit http://download.swift-nav.com to\n" + \
        "download the newest version.\n\n" + \
        "Local Console Version :\n\t" + \
            CONSOLE_VERSION + \
        "\nNewest Console Version :\n\t" + \
            index['piksi_v2.3.1']['console']['version'] + "\n"

    # Do local versions match latest from website?
    local_stm_version = parse_version(
        self.settings['system_info']['firmware_version'].value)
    remote_stm_version = parse_version(
        index['piksi_v2.3.1']['stm_fw']['version'])

    local_nap_version = parse_version(
        self.settings['system_info']['nap_version'].value)
    remote_nap_version = parse_version(
        index['piksi_v2.3.1']['nap_fw']['version'])

    self.fw_outdated = remote_nap_version > local_nap_version or \
                       remote_stm_version > local_stm_version

    local_console_version = parse_version(CONSOLE_VERSION)
    remote_console_version = parse_version(
        index['piksi_v2.3.1']['console']['version'])
    self.console_outdated = remote_console_version > local_console_version

    # Get firmware files from Swift Nav's website.
    try:
      f = urlopen(index['piksi_v2.3.1']['nap_fw']['url'])
      self.nap_ihx = IntelHex(f)
      f.close()
    except URLError:
      print "\nError: Failed to download latest Piksi SwiftNAP firmware from Swift Navigation's website (%s). Please visit our website to check that you're running the latest firmware.\n" % index['piksi_v2.3.1']['nap_fw']['url']

    try:
      f = urlopen(index['piksi_v2.3.1']['stm_fw']['url'])
      self.stm_ihx = IntelHex(f)
      f.close()
    except URLError:
      print "\nError: Failed to download latest Piksi STM firmware from Swift Navigation's website (%s). Please visit our website to check that you're running the latest firmware.\n" % index['piksi_v2.3.1']['stm_fw']['url']

    # Prompt user to update firmware. Only allow update if we successfully
    # downloaded both files.
    if self.prompt and self.fw_outdated and self.stm_ihx and self.nap_ihx:
      fw_update_prompt.run()

    # For timing aesthetics between windows popping up.
    sleep(0.5)

    # Check if console is out of date and notify user if so.
    if self.prompt and self.console_outdated:
      console_outdated_prompt.run()

  # Executed in GUI thread, called from Handler.
  def manage_firmware_updates(self):
    self.updating = True

    print "\n"

    # Flash NAP.
    print "Updating SwiftNAP firmware...\n"
    self.update_flash(self.nap_ihx, "M25")

    # Flash STM.
    print "Updating STM firmware...\n"
    self.update_flash(self.stm_ihx, "STM")

    # Piksi needs to jump to application after updating firmware.
    self.link.send_message(ids.BOOTLOADER_JUMP_TO_APP, '\x00')
    print "Firmware updates finished.\n"

    self.updating = False

  def update_flash(self, ihx, flash_type):

    # Reset device if the application is running to put into bootloader mode.
    self.link.send_message(ids.RESET, '')

    piksi_bootloader = bootload.Bootloader(self.link)
    print "Waiting for bootloader handshake message from Piksi ...\n"
    piksi_bootloader.wait_for_handshake()
    piksi_bootloader.reply_handshake()
    print "received bootloader handshake message.\n"
    print "Piksi Onboard Bootloader Version: " + piksi_bootloader.version + "\n"

    piksi_flash = flash.Flash(self.link, flash_type)
    piksi_flash.write_ihx(ihx, sys.stdout, mod_print = 0x10)
    piksi_flash.stop()

    piksi_bootloader.stop()


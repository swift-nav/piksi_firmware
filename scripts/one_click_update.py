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

from urllib2 import urlopen, URLError
from json import load as jsonload
from time import sleep
from intelhex import IntelHex
from new import instancemethod

from threading import Thread

from traits.api import HasTraits, Event, String
from traitsui.api import View, Handler, Action, Item, TextEditor
from pyface.api import GUI

from version import VERSION as CONSOLE_VERSION
import bootload
import sbp_piksi as ids
import flash

INDEX_URL = 'http://download.swift-nav.com/index.json'

# Handler methods that can be associated with buttons.
def set_execute_callback_true(self, info):
  info.object.execute_callback = True
  info.object.handler_executed = True

def set_execute_callback_false(self, info):
  info.object.execute_callback = False
  info.object.handler_executed = True

update_button = Action(name = "Update", action = "set_execute_callback_true", \
                             show_label=False)
close_button = Action(name = "Close", action = "set_execute_callback_false", \
                             show_label=False)

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
                     height=250,
                     width=450,
                     resizable=True
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


class OneClickUpdate():

  def __init__(self, link, output):
    self.link = link
    self.thread = None
    self.output = output
    self.settings = {}

  # Instead of inheriting Thread so start can be called multiple times.
  # Expectation is that OneClickUpdate.start is passed to SettingsView to
  # be called after settings are read out, which can happen multiple times.
  # Executed in GUI thread.
  #
  # Returns without any action taken on any calls after the first call.
  def start(self):
    if self.thread:
      return
    self.thread = Thread(target=self.run)
    self.thread.start()

  def point_to_settings(self, settings):
    self.settings = settings

  def write(self, text):
    GUI.invoke_later(self.output.write, text)
    GUI.process_events()

  # Executed in it's own thread.
  def run(self):

    # Create prompt objects.
    fw_update_prompt = \
        UpdatePrompt(
                     title='Firmware Update',
                     actions=[update_button, close_button],
                     update_callback=self.manage_firmware_updates,
                    )

    console_outdated_prompt = \
        UpdatePrompt(
                     title="Console Outdated",
                     actions=[close_button],
                    )

    # Get index that contains file URLs and latest
    # version strings from Swift Nav's website.
    try:
      f = urlopen(INDEX_URL)
      index = jsonload(f)
      f.close()
    except URLError:
      self.write("\nError: Failed to download latest file index from Swift Navigation's website (%s). Please visit our website to check that you're running the latest Piksi firmware and Piksi console.\n\n" % INDEX_URL)
      return

    # Make sure index contains all keys we are interested in.
    try:
      index['piksi_v2.3.1']['stm_fw']['version']
      index['piksi_v2.3.1']['stm_fw']['url']
      index['piksi_v2.3.1']['nap_fw']['version']
      index['piksi_v2.3.1']['nap_fw']['url']
      index['piksi_v2.3.1']['console']['version']
    except KeyError:
      self.write("\nError: Index downloaded from Swift Navigation's website (%s) doesn't contain all keys. Please contact Swift Navigation.\n\n" % INDEX_URL)
      return

    # Make sure settings contains Piksi firmware version strings.
    try:
      self.settings['system_info']['firmware_version'].value
      self.settings['system_info']['nap_version'].value
    except:
      self.write("\nError: Settings received from Piksi don't contain firmware version keys. Please contact Swift Navigation.\n\n" % INDEX_URL)
      return

    # Assign text to UpdatePrompt's
    fw_update_prompt.text = \
        "Local STM Version :\n\t%s\n" % \
            self.settings['system_info']['firmware_version'].value + \
        "Newest STM Version :\n\t%s\n\n" % \
            index['piksi_v2.3.1']['stm_fw']['version'] + \
        "Local SwiftNAP Version :\n\t%s\n" % \
            self.settings['system_info']['nap_version'].value + \
        "Newest SwiftNAP Version :\n\t%s\n\n" % \
            index['piksi_v2.3.1']['nap_fw']['version']

    console_outdated_prompt.text = \
        "Your Console is out of date and may be incompatible\n" + \
        "with current firmware. We highly recommend upgrading\n" + \
        "to ensure proper behavior.\n\n" + \
        "Please visit http://download.swift-nav.com to\n" + \
        "download the newest version.\n\n" + \
        "Local Console Version :\n\t" + \
            CONSOLE_VERSION + \
        "\nNewest Console Version :\n\t" + \
            index['piksi_v2.3.1']['console']['version'] + "\n"

    # Do local version match latest from website?
    self.stm_fw_outdated = index['piksi_v2.3.1']['stm_fw']['version'] \
                               !=  self.settings['system_info']['firmware_version'].value
    self.nap_fw_outdated = index['piksi_v2.3.1']['nap_fw']['version'] \
                               !=  self.settings['system_info']['nap_version'].value
    self.console_outdated = index['piksi_v2.3.1']['console']['version'] \
                               !=  CONSOLE_VERSION

    # Get firmware files from Swift Nav's website.
    self.nap_ihx = None
    if self.nap_fw_outdated:
      try:
        f = urlopen(index['piksi_v2.3.1']['nap_fw']['url'])
        self.nap_ihx = IntelHex(f)
        f.close()
      except URLError:
        self.write("\nError: Failed to download latest Piksi SwiftNAP firmware from Swift Navigation's website (%s). Please visit our website to check that you're running the latest firmware.\n" % index['piksi_v2.3.1']['nap_fw']['url'])

    self.stm_ihx = None
    if self.stm_fw_outdated:
      try:
        f = urlopen(index['piksi_v2.3.1']['stm_fw']['url'])
        self.stm_ihx = IntelHex(f)
        f.close()
      except URLError:
        self.write("\nError: Failed to download latest Piksi STM firmware from Swift Navigation's website (%s). Please visit our website to check that you're running the latest firmware.\n" % index['piksi_v2.3.1']['stm_fw']['url'])

    # Prompt user to update firmware(s). Only update if firmware was
    # successfully downloaded. If both are out of date, only allow
    # update if we successfully downloaded both files.
    if (self.stm_fw_outdated and self.stm_ihx and not self.nap_fw_outdated) or \
        (self.nap_fw_outdated and self.nap_ihx and not self.stm_fw_outdated) or \
         (self.stm_fw_outdated and self.stm_ihx and \
           self.nap_fw_outdated and self.nap_ihx):
      fw_update_prompt.run()

    # For timing aesthetics between windows popping up.
    sleep(0.5)

    # Check if console is out of date and notify user if so.
    if self.console_outdated:
      console_outdated_prompt.run()

  # Executed in GUI thread, called from Handler.
  def manage_firmware_updates(self):

    self.write("\n")

    # Flash NAP if outdated.
    if self.nap_fw_outdated:
      self.write("Updating SwiftNAP firmware...\n")
      self.update_firmware(self.nap_ihx, "M25")

    # Flash STM if outdated.
    if self.stm_fw_outdated:
      self.write("Updating STM firmware...\n")
      self.update_firmware(self.stm_ihx, "STM")

    # Piksi needs to jump to application if we updated either firmware.
    if self.nap_fw_outdated or self.stm_fw_outdated:
      self.link.send_message(ids.BOOTLOADER_JUMP_TO_APP, '\x00')
      self.write("Firmware updates finished.\n")

  def update_firmware(self, ihx, flash_type):

    # Reset device if the application is running to put into bootloader mode.
    self.link.send_message(ids.RESET, '')

    self.write("\n")

    piksi_bootloader = bootload.Bootloader(self.link)
    self.write("Waiting for bootloader handshake message from Piksi ...\n")
    piksi_bootloader.wait_for_handshake()
    piksi_bootloader.reply_handshake()
    self.write("received bootloader handshake message.\n")
    self.write("Piksi Onboard Bootloader Version: " + piksi_bootloader.version + "\n")

    piksi_flash = flash.Flash(self.link, flash_type)
    piksi_flash.write_ihx(ihx, self.output, mod_print = 0x10)
    piksi_flash.stop()

    piksi_bootloader.stop()

    self.write("\n")


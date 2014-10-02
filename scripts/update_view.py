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
# TODO: blank out update, choose firmware updates.
# TODO: have Firmware Update tab blink if new firmware is available
# TODO: add button to wipe flash upon update (default on)
# TODO: ability to flash piksi if firmware is borked - prompt user to reset?
# TODO: remove automatic download, just prompt user to press download button

from urllib2 import urlopen, URLError
from urlparse import urlparse
from json import load as jsonload
from time import sleep
from intelhex import IntelHex, HexRecordError, HexReaderError
from pkg_resources import parse_version

from threading import Thread

from traits.api import HasTraits, Event, String, Button, Instance
from traitsui.api import View, Handler, Action, Item, TextEditor, VGroup, \
                         UItem, InstanceEditor, VSplit, HSplit, HGroup
from pyface.api import GUI, FileDialog, OK

from version import VERSION as CONSOLE_VERSION
import bootload
import sbp_piksi as ids
import flash
import callback_prompt as prompt

from output_stream import OutputStream

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

class IntelHexFileDialog(HasTraits):

  file_wildcard = String("Intel HEX File (*.hex)|*.hex|All files|*")

  filename = String('Please choose a file')
  choose_fw = Button(label='Choose Firmware File')
  view = View(
               UItem('filename'),
               UItem('choose_fw')
             )

  def __init__(self, flash_type):
    if not flash_type=='M25' and not flash_type=='STM':
      raise ValueError("flash_type must be 'M25' or 'STM'")
    self._flash_type = flash_type
    self.ihx = None

  def load_ihx(self, filepath):
    try:
      self.ihx = IntelHex(filepath)
      self.filename = os.path.split(filepath)[1]
    except HexRecordError:
      self.ihx = None
      self.filename = 'Error: File is not a valid Intel HEX File'

    # Check that address ranges are valid for self._flash_type.
    ihx_addrs = flash.ihx_ranges(self.ihx)
    if self._flash_type == "M25":
      try:
        sectors = flash.sectors_used(ihx_addrs, flash.m25_addr_sector_map)
      except IndexError:
        self.filename = 'Error: HEX File contains restricted address ' + \
                        '(STM Firmware File Chosen?)'
        self.ihx = None
    elif self._flash_type == "STM":
      try:
        sectors = flash.sectors_used(ihx_addrs, flash.stm_addr_sector_map)
      except:
        self.filename = 'Error: HEX File contains restricted address ' + \
                        '(NAP Firmware File Chosen?)'
        self.ihx = None

  def _choose_fw_fired(self):
    dialog = FileDialog(label='Choose Firmware File',
                        action='open', wildcard=self.file_wildcard)
    dialog.open()
    if dialog.return_code == OK:
      self.filename = dialog.filename
      filepath = os.path.join(dialog.directory, dialog.filename)
      self.load_ihx(filepath)
    else:
      self.filename = 'Error while selecting file'

# Save file at URL to current directory.
def _download_url_file(url):
  url = url.encode('ascii')
  urlpath = urlparse(url).path
  filename = os.path.split(urlpath)[1]

  url_file = urlopen(url)
  lines = url_file.readlines()
  with open(filename, 'w') as f:
    for line in lines:
      f.write(line)
  url_file.close()

  return filename

class UpdateView(HasTraits):

  piksi_stm_vers = String('Waiting for Piksi to send settings...')
  newest_stm_vers = String('Waiting for newest firmware info...')
  piksi_nap_vers = String('Waiting for Piksi to send settings...')
  newest_nap_vers = String('Waiting for newest firmware info...')
  update_firmware = Button(label='Update Piksi Firmware')
  #download_firmware = Button(label='Download Newest Firmware Files')

  stm_fw = Instance(IntelHexFileDialog)
  nap_fw = Instance(IntelHexFileDialog)

  stream = Instance(OutputStream)

  view = View(
    VGroup(
      HGroup(
        VGroup(
          Item('piksi_stm_vers', label='Piksi STM Firmware Version'),
          Item('newest_stm_vers', label='Newest STM Firmware Version'),
          Item('piksi_nap_vers', label='Piksi NAP Firmware Version'),
          Item('newest_nap_vers', label='Newest NAP Firmware Version'),
        ),
        VGroup(
          Item('stm_fw', style='custom', label='STM Firmware File'),
          Item('nap_fw', style='custom', label='NAP Firmware File'),
        ),
      ),
      #UItem('download_firmware'),
      UItem('update_firmware'),
      Item(
        'stream',
        style='custom',
        editor=InstanceEditor(),
        label='Update Status',
      ),
    )
  )

  def __init__(self, link, prompt=True):
    self.link = link
    self.settings = {}
    self.prompt = prompt
    self.updating = False
    self.python_console_cmds = {
      'update': self
    }
    self.stm_fw = IntelHexFileDialog('STM')
    self.nap_fw = IntelHexFileDialog('M25')
    self.stream = OutputStream()

  def write(self, text):
    self.stream.write(text)
    self.stream.flush()

  def _update_firmware_fired(self):
    if not self.updating:
      if self.nap_fw.ihx and self.stm_fw.ihx:
        GUI.invoke_later(self.manage_firmware_updates)
      else:
        # TODO: change this for a warning window
        self.write("No firmware files have been chosen yet!")
    else:
      self.write("Already updating firmware")

  def _download_firmware(self):
    self.nap_fw.ihx = None
    self.nap_fw.filename = 'Downloading latest firmware...'
    self.stm_fw.ihx = None
    self.stm_fw.filename = 'Downloading latest firmware...'

    # Get firmware files from Swift Nav's website, save to disk, and load.
    try:
      url = self.index['piksi_v2.3.1']['nap_fw']['url']
      filepath = _download_url_file(url)
      self.nap_fw.load_ihx(filepath)
    except AttributeError:
      self.nap_fw.filename = "Error downloading firmware: index file not downloaded yet"
    except KeyError:
      self.nap_fw.filename = "Error downloading firmware: URL not present in index"
    except URLError:
      self.nap_fw.filename = "Error: Failed to download latest NAP firmware from Swift Navigation's website"

    try:
      url = self.index['piksi_v2.3.1']['stm_fw']['url']
      filepath = _download_url_file(url)
      self.stm_fw.load_ihx(filepath)
    except AttributeError:
      self.stm_fw.filename = "Error downloading firmware: index file not downloaded yet"
    except KeyError:
      self.stm_fw.filename = "Error downloading firmware: URL not present in index"
    except URLError:
      self.stm_fw.filename = "Error: Failed to download latest STM firmware from Swift Navigation's website"

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

  # Executed in it's own thread.
  def run(self):

    # Create prompt objects.
    fw_update_prompt = \
        prompt.CallbackPrompt(
                              title='Firmware Update',
                              actions=[prompt.close_button]
                             )

    console_outdated_prompt = \
        prompt.CallbackPrompt(
                              title="Piksi Console Outdated",
                              actions=[prompt.close_button],
                             )

    # Check that settings received from Piksi contain FW versions.
    try:
      self.piksi_stm_vers = \
        self.settings['system_info']['firmware_version'].value
      self.piksi_nap_vers = \
        self.settings['system_info']['nap_version'].value
    except KeyError:
      self.write("\nError: Settings received from Piksi don't contain firmware version keys. Please contact Swift Navigation.\n\n")
      return

    # Get index that contains file URLs and latest
    # version strings from Swift Nav's website.
    try:
      f = urlopen(INDEX_URL)
      self.index = jsonload(f)
      f.close()
    except URLError:
      self.write("\nError: Failed to download latest file index from Swift Navigation's website (%s). Please visit our website to check that you're running the latest Piksi firmware and Piksi console.\n\n" % INDEX_URL)
      return

    # Make sure index contains all keys we are interested in.
    try:
      self.newest_stm_vers = self.index['piksi_v2.3.1']['stm_fw']['version']
      self.newest_nap_vers = self.index['piksi_v2.3.1']['nap_fw']['version']
      self.index['piksi_v2.3.1']['stm_fw']['url']
      self.index['piksi_v2.3.1']['nap_fw']['url']
      self.index['piksi_v2.3.1']['console']['version']
    except KeyError:
      self.write("\nError: Index downloaded from Swift Navigation's website (%s) doesn't contain all keys. Please contact Swift Navigation.\n\n" % INDEX_URL)
      return

    # Assign text to CallbackPrompt's
    fw_update_prompt.text = \
        "New Piksi firmware available.\n" + \
        "Please use the Firmware Update tab to update.\n\n" + \
        "Newest STM Version :\n\t%s\n\n" % \
            self.index['piksi_v2.3.1']['stm_fw']['version'] + \
        "Newest SwiftNAP Version :\n\t%s\n\n" % \
            self.index['piksi_v2.3.1']['nap_fw']['version']

    console_outdated_prompt.text = \
        "Your Piksi Console is out of date and may be incompatible\n" + \
        "with current firmware. We highly recommend upgrading to\n" + \
        "ensure proper behavior.\n\n" + \
        "Please visit http://download.swift-nav.com to\n" + \
        "download the newest version.\n\n" + \
        "Local Console Version :\n\t" + \
            CONSOLE_VERSION + \
        "\nNewest Console Version :\n\t" + \
            self.index['piksi_v2.3.1']['console']['version'] + "\n"

    # Do local versions match latest from website?
    local_stm_version = parse_version(
        self.settings['system_info']['firmware_version'].value)
    remote_stm_version = parse_version(
        self.index['piksi_v2.3.1']['stm_fw']['version'])

    local_nap_version = parse_version(
        self.settings['system_info']['nap_version'].value)
    remote_nap_version = parse_version(
        self.index['piksi_v2.3.1']['nap_fw']['version'])

    self.fw_outdated = remote_nap_version > local_nap_version or \
                       remote_stm_version > local_stm_version

    local_console_version = parse_version(CONSOLE_VERSION)
    remote_console_version = parse_version(
        self.index['piksi_v2.3.1']['console']['version'])
    self.console_outdated = remote_console_version > local_console_version

    # Get firmware files from Swift Nav's website.
    self._download_firmware()
    
    # Prompt user to update firmware. Only allow update if we successfully
    # downloaded both files.
    if self.prompt and self.fw_outdated and self.stm_fw.ihx and self.nap_fw.ihx:
      fw_update_prompt.run()

    # For timing aesthetics between windows popping up.
    sleep(0.5)

    # Check if console is out of date and notify user if so.
    if self.prompt and self.console_outdated:
      console_outdated_prompt.run()

  # Executed in GUI thread, called from Handler.
  def manage_firmware_updates(self):
    self.updating = True

    self.write("\n")

    # Flash NAP.
    self.write("Updating SwiftNAP firmware...\n")
    self.update_flash(self.nap_fw.ihx, "M25")

    # Flash STM.
    self.write("Updating STM firmware...\n")
    self.update_flash(self.stm_fw.ihx, "STM")

    # Piksi needs to jump to application after updating firmware.
    self.link.send_message(ids.BOOTLOADER_JUMP_TO_APP, '\x00')
    self.write("Firmware updates finished.\n")

    self.updating = False

  def update_flash(self, ihx, flash_type):

    # Reset device if the application is running to put into bootloader mode.
    self.link.send_message(ids.RESET, '')

    piksi_bootloader = bootload.Bootloader(self.link)
    self.write("Waiting for bootloader handshake message from Piksi ...\n")
    piksi_bootloader.wait_for_handshake()
    piksi_bootloader.reply_handshake()
    self.write("received bootloader handshake message.\n")
    self.write("Piksi Onboard Bootloader Version: " + piksi_bootloader.version + "\n")

    piksi_flash = flash.Flash(self.link, flash_type)
    piksi_flash.write_ihx(ihx, self.stream, mod_print = 0x10)
    piksi_flash.stop()

    piksi_bootloader.stop()


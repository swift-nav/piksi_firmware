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

# TODO: ability to flash piksi if firmware is borked - prompt user to reset?

from urllib2 import urlopen, URLError
from urlparse import urlparse
from json import load as jsonload
from time import sleep
from intelhex import IntelHex, HexRecordError, HexReaderError
from pkg_resources import parse_version

from threading import Thread

from traits.api import HasTraits, Event, String, Button, Instance, Int, Bool, \
                       on_trait_change
from traitsui.api import View, Handler, Action, Item, TextEditor, VGroup, \
                         UItem, InstanceEditor, VSplit, HSplit, HGroup, \
                         BooleanEditor
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

  status = String('Please choose a file')
  choose_fw = Button(label='Choose Firmware File')
  view = View(
               UItem('status'),
               UItem('choose_fw')
             )

  def __init__(self, flash_type):
    if not flash_type=='M25' and not flash_type=='STM':
      raise ValueError("flash_type must be 'M25' or 'STM'")
    self._flash_type = flash_type
    self.ihx = None

  def set_status(self, status):
    self.ihx = None
    self.status = status

  def load_ihx(self, filepath):
    try:
      self.ihx = IntelHex(filepath)
      self.status = os.path.split(filepath)[1]
    except HexRecordError:
      self.set_status('Error: File is not a valid Intel HEX File')

    # Check that address ranges are valid for self._flash_type.
    ihx_addrs = flash.ihx_ranges(self.ihx)
    if self._flash_type == "M25":
      try:
        sectors = flash.sectors_used(ihx_addrs, flash.m25_addr_sector_map)
      except IndexError:
        self.set_status('Error: HEX File contains restricted address ' + \
                        '(STM Firmware File Chosen?)')
    elif self._flash_type == "STM":
      try:
        sectors = flash.sectors_used(ihx_addrs, flash.stm_addr_sector_map)
      except:
        self.set_status('Error: HEX File contains restricted address ' + \
                        '(NAP Firmware File Chosen?)')

  def _choose_fw_fired(self):
    dialog = FileDialog(label='Choose Firmware File',
                        action='open', wildcard=self.file_wildcard)
    dialog.open()
    if dialog.return_code == OK:
      filepath = os.path.join(dialog.directory, dialog.filename)
      self.load_ihx(filepath)
    else:
      self.set_status('Error while selecting file')

class UpdateView(HasTraits):

  piksi_stm_vers = String('Waiting for Piksi to send settings...')
  newest_stm_vers = String('Waiting for Newest Firmware info...')
  piksi_nap_vers = String('Waiting for Piksi to send settings...')
  newest_nap_vers = String('Waiting for Newest Firmware info...')

  erase_stm = Bool(True)

  update_firmware = Button(label='Update Piksi Firmware')
  updating = Bool(False)
  update_en = Bool(False)

  download_firmware = Button(label='Download Newest Firmware Files')
  downloading = Bool(False)
  download_fw_en = Bool(True)

  stm_fw = Instance(IntelHexFileDialog)
  nap_fw = Instance(IntelHexFileDialog)
  choose_en = Bool(True)

  stream = Instance(OutputStream)

  view = View(
    VGroup(
      HGroup(
        VGroup(
          Item('piksi_stm_vers', label='Piksi STM Firmware Version'),
          Item('newest_stm_vers', label='Newest STM Firmware Version'),
          Item('piksi_nap_vers', label='Piksi NAP Firmware Version'),
          Item('newest_nap_vers', label='Newest NAP Firmware Version'),
          Item('erase_stm', label='Erase Entire STM flash'),
        ),
        VGroup(
          Item('stm_fw', style='custom', label='STM Firmware File', \
               enabled_when='choose_en'),
          Item('nap_fw', style='custom', label='NAP Firmware File', \
               enabled_when='choose_en'),
        ),
      ),
      UItem('download_firmware', enabled_when='download_fw_en'),
      UItem('update_firmware', enabled_when='update_en'),
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
    self.python_console_cmds = {
      'update': self

    }
    self.stm_fw = IntelHexFileDialog('STM')
    self.stm_fw.on_trait_change(self._manage_enables, 'status')
    self.nap_fw = IntelHexFileDialog('M25')
    self.nap_fw.on_trait_change(self._manage_enables, 'status')
    self.stream = OutputStream()

  def _manage_enables(self):
    if self.updating == True or self.downloading == True:
      self.update_en = False
    else:
      if self.stm_fw.ihx != None and self.nap_fw.ihx != None:
        self.update_en = True
      else:
        self.update_en = False

    if self.updating == True or self.downloading == True:
      self.download_fw_en = False
      self.choose_en = False
    else:
      self.download_fw_en = True
      self.choose_en = True

  def _updating_changed(self):
    self._manage_enables()

  def _downloading_changed(self):
    self._manage_enables()

  def _write(self, text):
    self.stream.write(text)
    self.stream.write('\n')
    self.stream.flush()

  def _update_firmware_fired(self):
    self._write('')
    GUI.invoke_later(self.manage_firmware_updates)

  # Save file at URL to current directory.
  def _download_file_from_url(self, url):
    url = url.encode('ascii')
    urlpath = urlparse(url).path
    filename = os.path.split(urlpath)[1]

    self._write('Downloading file from %s' % url)

    url_file = urlopen(url, timeout=120)
    lines = url_file.readlines()
    with open(filename, 'w') as f:
      for line in lines:
        f.write(line)
    url_file.close()

    self._write('Saved file to %s' % os.path.abspath(filename))

    return filename

  def _download_firmware(self):
    self.downloading = True

    status = 'Downloading Newest Firmware...'
    self.nap_fw.set_status(status)
    self.stm_fw.set_status(status)
    self._write(status)

    # Get firmware files from Swift Nav's website, save to disk, and load.
    try:
      url = self.index['piksi_v2.3.1']['nap_fw']['url']
      filepath = self._download_file_from_url(url)
      self.nap_fw.load_ihx(filepath)
    except AttributeError:
      self.nap_fw.set_status("Error downloading firmware: index file not downloaded yet")
      self._write("Error downloading firmware: index file not downloaded yet")
    except KeyError:
      self.nap_fw.set_status("Error downloading firmware: URL not present in index")
      self._write("Error downloading firmware: URL not present in index")
    except URLError:
      self.nap_fw.set_status("Error: Failed to download latest NAP firmware from Swift Navigation's website")
      self._write("Error: Failed to download latest NAP firmware from Swift Navigation's website")

    try:
      url = self.index['piksi_v2.3.1']['stm_fw']['url']
      filepath = self._download_file_from_url(url)
      self.stm_fw.load_ihx(filepath)
    except AttributeError:
      self.stm_fw.set_status("Error downloading firmware: index file not downloaded yet")
      self._write("Error downloading firmware: index file not downloaded yet")
    except KeyError:
      self.stm_fw.set_status("Error downloading firmware: URL not present in index")
      self._write("Error downloading firmware: URL not present in index")
    except URLError:
      self.stm_fw.set_status("Error: Failed to download latest STM firmware from Swift Navigation's website")
      self._write("Error: Failed to download latest STM firmware from Swift Navigation's website")

    self.downloading = False

  def _download_firmware_fired(self):
    try:
      if self._download_firmware_thread.is_alive():
        return
    except AttributeError:
      pass

    self._write('')

    self._download_firmware_thread = Thread(target=self._download_firmware)
    self._download_firmware_thread.start()

  # Instantiate an instance variable thread instead of inheriting Thread 
  # so start can be called multiple times.
  # Expectation is that OneClickUpdate.start is passed to SettingsView to
  # be called after settings are read out, which can happen multiple times.
  # This method is executed in GUI thread.
  def start(self):

    try:
      if self.update_thread.is_alive():
        return
    except AttributeError:
      pass

    self.update_thread = Thread(target=self.check_for_updates)
    self.update_thread.start()

  # Executed in it's own thread.
  def check_for_updates(self):
    
    # Check that settings received from Piksi contain FW versions.
    try:
      self.piksi_stm_vers = \
        self.settings['system_info']['firmware_version'].value
      self.piksi_nap_vers = \
        self.settings['system_info']['nap_version'].value
    except KeyError:
      self._write("\nError: Settings received from Piksi don't contain firmware version keys. Please contact Swift Navigation.\n")
      return

    # Get index that contains file URLs and latest
    # version strings from Swift Nav's website.
    try:
      f = urlopen(INDEX_URL, timeout=120)
      self.index = jsonload(f)
      f.close()
    except URLError:
      self._write("\nError: Failed to download latest file index from Swift Navigation's website (%s). Please visit our website to check that you're running the latest Piksi firmware and Piksi console.\n" % INDEX_URL)
      return

    # Make sure index contains all keys we are interested in.
    try:
      self.newest_stm_vers = self.index['piksi_v2.3.1']['stm_fw']['version']
      self.newest_nap_vers = self.index['piksi_v2.3.1']['nap_fw']['version']
      self.index['piksi_v2.3.1']['stm_fw']['url']
      self.index['piksi_v2.3.1']['nap_fw']['url']
      self.index['piksi_v2.3.1']['console']['version']
    except KeyError:
      self._write("\nError: Index downloaded from Swift Navigation's website (%s) doesn't contain all keys. Please contact Swift Navigation.\n" % INDEX_URL)
      return
    
    # Check if firmware is out of date and notify user if so.
    if self.prompt:
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

      if self.fw_outdated:
        fw_update_prompt = \
            prompt.CallbackPrompt(
                                  title='Firmware Update',
                                  actions=[prompt.close_button]
                                 )

        fw_update_prompt.text = \
            "New Piksi firmware available.\n\n" + \
            "Please use the Firmware Update tab to update.\n\n" + \
            "Newest STM Version :\n\t%s\n\n" % \
                self.index['piksi_v2.3.1']['stm_fw']['version'] + \
            "Newest SwiftNAP Version :\n\t%s\n\n" % \
                self.index['piksi_v2.3.1']['nap_fw']['version']

        fw_update_prompt.run()

    # For timing aesthetics between windows popping up.
    sleep(0.5)

    # Check if console is out of date and notify user if so.
    if self.prompt:
      local_console_version = parse_version(CONSOLE_VERSION)
      remote_console_version = parse_version(
          self.index['piksi_v2.3.1']['console']['version'])
      self.console_outdated = remote_console_version > local_console_version

      if self.console_outdated:
        console_outdated_prompt = \
            prompt.CallbackPrompt(
                                  title="Piksi Console Outdated",
                                  actions=[prompt.close_button],
                                 )

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

        console_outdated_prompt.run()

  # Executed in GUI thread, called from Handler.
  def manage_firmware_updates(self):
    self.updating = True

    # Erase STM if so directed.
    if self.erase_stm:
      self._write("Erasing STM flash...")
      self.erase_flash("STM")
      self._write("")

    # Flash STM.
    self._write("Updating STM firmware...")
    self.update_flash(self.stm_fw.ihx, "STM")
    self._write("")

    # Flash NAP.
    self._write("Updating SwiftNAP firmware...")
    self.update_flash(self.nap_fw.ihx, "M25")
    self._write("")

    # Must tell Piksi to jump to application after updating firmware.
    self.link.send_message(ids.BOOTLOADER_JUMP_TO_APP, '\x00')
    self._write("Firmware updates finished.")
    self._write("")

    self.updating = False

  def erase_flash(self, flash_type):

    # Reset device if the application is running to put into bootloader mode.
    self.link.send_message(ids.RESET, '')

    pk_boot = bootload.Bootloader(self.link)
    self._write("Waiting for bootloader handshake message from Piksi ...")
    pk_boot.wait_for_handshake()
    pk_boot.reply_handshake()
    self._write("received bootloader handshake message.")
    self._write("Piksi Onboard Bootloader Version: " + pk_boot.version)

    pk_flash = flash.Flash(self.link, flash_type)

    sectors_to_erase = \
        set(range(pk_flash.n_sectors)).difference(set(pk_flash.restricted_sectors))
    for s in sorted(sectors_to_erase):
      self._write('Erasing %s sector %d' % (pk_flash.flash_type,s))
      pk_flash.erase_sector(s)

    pk_flash.stop()
    pk_boot.stop()

  def update_flash(self, ihx, flash_type):

    # Reset device if the application is running to put into bootloader mode.
    self.link.send_message(ids.RESET, '')

    pk_boot = bootload.Bootloader(self.link)
    self._write("Waiting for bootloader handshake message from Piksi ...")
    pk_boot.wait_for_handshake()
    pk_boot.reply_handshake()
    self._write("received bootloader handshake message.")
    self._write("Piksi Onboard Bootloader Version: " + pk_boot.version)

    pk_flash = flash.Flash(self.link, flash_type)
    pk_flash.write_ihx(ihx, self.stream, mod_print = 0x10)
    pk_flash.stop()

    pk_boot.stop()


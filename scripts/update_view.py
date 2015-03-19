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

from urllib2 import URLError
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
from pyface.api import GUI, FileDialog, OK, ProgressDialog

from version import VERSION as CONSOLE_VERSION
import bootload
import sbp_piksi as ids
import flash
import callback_prompt as prompt
from update_downloader import UpdateDownloader
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

INDEX_URL = 'http://downloads.swiftnav.com/index.json'


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

class PulsableProgressDialog(ProgressDialog):

  def __init__(self, max, pulsed=False):
    super(PulsableProgressDialog, self).__init__()
    self.min = 0
    self.max = 0
    self.pulsed = pulsed
    self.passed_max = max

  def progress(self, count):
    # Provide user feedback initially via pulse for slow sector erases.
    if self.pulsed:
      if count > 20:
        self.max = self.passed_max
    else:
      self.max = self.passed_max
    self.update(count)

  def reset(self, max, pulsed=False):
    self.max = 0
    self.passed_max = max
    self.pulsed = pulsed
    self.progress(0)

class UpdateView(HasTraits):

  piksi_stm_vers = String('Waiting for Piksi to send settings...')
  newest_stm_vers = String('Downloading Newest Firmware info...')
  piksi_nap_vers = String('Waiting for Piksi to send settings...')
  newest_nap_vers = String('Downloading Newest Firmware info...')
  local_console_vers = String(CONSOLE_VERSION)
  newest_console_vers = String('Downloading Newest Console info...')

  erase_stm = Bool(True)
  erase_en = Bool(True)

  update_firmware = Button(label='Update Piksi Firmware')
  updating = Bool(False)
  update_en = Bool(False)

  download_firmware = Button(label='Download Newest Firmware Files')
  downloading = Bool(False)
  download_fw_en = Bool(True)

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
          Item('local_console_vers', label='Local Piksi Console Version'),
          Item('newest_console_vers', label='Newest Piksi Console Version'),
        ),
        VGroup(
          Item('stm_fw', style='custom', label='STM Firmware File', \
               enabled_when='download_fw_en'),
          Item('nap_fw', style='custom', label='NAP Firmware File', \
               enabled_when='download_fw_en'),
          Item('erase_stm', label='Erase Entire STM flash', \
               enabled_when='erase_en'),
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
    self.update_dl = None
    self.stm_fw = IntelHexFileDialog('STM')
    self.stm_fw.on_trait_change(self._manage_enables, 'status')
    self.nap_fw = IntelHexFileDialog('M25')
    self.nap_fw.on_trait_change(self._manage_enables, 'status')
    self.stream = OutputStream()
    self.get_latest_version_info()

  def _manage_enables(self):
    if self.updating == True or self.downloading == True:
      self.update_en = False
      self.download_fw_en = False
    else:
      self.download_fw_en = True
      if self.stm_fw.ihx != None and self.nap_fw.ihx != None:
        self.update_en = True
      else:
        self.update_en = False

    if self.updating == True:
      self.erase_en = False
    else:
      self.erase_en = True

  def _updating_changed(self):
    self._manage_enables()

  def _downloading_changed(self):
    self._manage_enables()

  def _write(self, text):
    self.stream.write(text)
    self.stream.write('\n')
    self.stream.flush()

  def _update_firmware_fired(self):
    try:
      if self._firmware_update_thread.is_alive():
        return
    except AttributeError:
      pass

    self._firmware_update_thread = Thread(target=self.manage_firmware_updates)
    self._firmware_update_thread.start()

  def _download_firmware(self):

    self._write('')

    # Check that we received the index file from the website.
    if self.update_dl == None:
      self._write("Error: Can't download firmware files")
      return

    self.downloading = True

    status = 'Downloading Newest Firmware...'
    self.nap_fw.set_status(status)
    self.stm_fw.set_status(status)
    self._write(status)

    # Get firmware files from Swift Nav's website, save to disk, and load.
    try:
      self._write('Downloading Newest NAP firmware')
      filepath = self.update_dl.download_nap_firmware()
      self._write('Saved file to %s' % filepath)
      self.nap_fw.load_ihx(filepath)
    except AttributeError:
      self.nap_fw.set_status("Error downloading firmware")
      self._write("Error downloading firmware: index file not downloaded yet")
    except KeyError:
      self.nap_fw.set_status("Error downloading firmware")
      self._write("Error downloading firmware: URL not present in index")
    except URLError:
      self.nap_fw.set_status("Error downloading firmware")
      self._write("Error: Failed to download latest NAP firmware from Swift Navigation's website")

    try:
      self._write('Downloading Newest STM firmware')
      filepath = self.update_dl.download_stm_firmware()
      self._write('Saved file to %s' % filepath)
      self.stm_fw.load_ihx(filepath)
    except AttributeError:
      self.stm_fw.set_status("Error downloading firmware")
      self._write("Error downloading firmware: index file not downloaded yet")
    except KeyError:
      self.stm_fw.set_status("Error downloading firmware")
      self._write("Error downloading firmware: URL not present in index")
    except URLError:
      self.stm_fw.set_status("Error downloading firmware")
      self._write("Error: Failed to download latest STM firmware from Swift Navigation's website")

    self.downloading = False

  def _download_firmware_fired(self):
    try:
      if self._download_firmware_thread.is_alive():
        return
    except AttributeError:
      pass

    self._download_firmware_thread = Thread(target=self._download_firmware)
    self._download_firmware_thread.start()

  def compare_versions(self):
    try:
      if self._compare_versions_thread.is_alive():
        return
    except AttributeError:
      pass

    self._compare_versions_thread = Thread(target=self._compare_versions)
    self._compare_versions_thread.start()

  def _compare_versions(self):

    # Check that settings received from Piksi contain FW versions.
    try:
      self.piksi_stm_vers = \
        self.settings['system_info']['firmware_version'].value
      self.piksi_nap_vers = \
        self.settings['system_info']['nap_version'].value
    except KeyError:
      self._write("\nError: Settings received from Piksi don't contain firmware version keys. Please contact Swift Navigation.\n")
      return

    # Check that we received the index file from the website.
    if self.update_dl == None:
      self._write("Error: No website index to use to compare versions with local firmware")
      return

    # Check if console is out of date and notify user if so.
    if self.prompt:
      local_console_version = parse_version(CONSOLE_VERSION)
      remote_console_version = parse_version(self.newest_console_vers)
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
            "Please visit http://downloads.swiftnav.com to\n" + \
            "download the newest version.\n\n" + \
            "Local Console Version :\n\t" + \
                CONSOLE_VERSION + \
            "\nNewest Console Version :\n\t" + \
                self.update_dl.index['piksi_v2.3.1']['console']['version'] + "\n"

        console_outdated_prompt.run()

    # For timing aesthetics between windows popping up.
    sleep(0.5)

    # Check if firmware is out of date and notify user if so.
    if self.prompt:
      local_stm_version = parse_version(
          self.settings['system_info']['firmware_version'].value)
      remote_stm_version = parse_version(self.newest_stm_vers)

      local_nap_version = parse_version(
          self.settings['system_info']['nap_version'].value)
      remote_nap_version = parse_version(self.newest_nap_vers)

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
                self.update_dl.index['piksi_v2.3.1']['stm_fw']['version'] + \
            "Newest SwiftNAP Version :\n\t%s\n\n" % \
                self.update_dl.index['piksi_v2.3.1']['nap_fw']['version']

        fw_update_prompt.run()

  def get_latest_version_info(self):

    try:
      if self._get_latest_version_info_thread.is_alive():
        return
    except AttributeError:
      pass

    self._get_latest_version_info_thread = Thread(target=self._get_latest_version_info)
    self._get_latest_version_info_thread.start()

  def _get_latest_version_info(self):

    try:
      self.update_dl = UpdateDownloader()
    except URLError:
      self._write("\nError: Failed to download latest file index from Swift Navigation's website. Please visit our website to check that you're running the latest Piksi firmware and Piksi console.\n")
      return

    # Make sure index contains all keys we are interested in.
    try:
      self.newest_stm_vers = self.update_dl.index['piksi_v2.3.1']['stm_fw']['version']
      self.newest_nap_vers = self.update_dl.index['piksi_v2.3.1']['nap_fw']['version']
      self.newest_console_vers = self.update_dl.index['piksi_v2.3.1']['console']['version']
    except KeyError:
      self._write("\nError: Index downloaded from Swift Navigation's website (%s) doesn't contain all keys. Please contact Swift Navigation.\n" % INDEX_URL)
      return

  # Executed in GUI thread, called from Handler.
  def manage_firmware_updates(self):
    self.updating = True

    self._write('')

    # Erase STM if box is checked.
    if self.erase_stm:
      text = "Erasing STM"
      self._write(text)
      self.create_flash("STM")
      sectors_to_erase = \
          set(range(self.pk_flash.n_sectors)).difference(set(self.pk_flash.restricted_sectors))
      progress_dialog = PulsableProgressDialog(len(sectors_to_erase), False)
      progress_dialog.title = text
      GUI.invoke_later(progress_dialog.open)
      erase_count = 0
      for s in sorted(sectors_to_erase):
        progress_dialog.progress(erase_count)
        self._write('Erasing %s sector %d' % (self.pk_flash.flash_type,s))
        self.pk_flash.erase_sector(s)
        erase_count += 1
      self.stop_flash()
      self._write("")

    # Flash STM.
    text = "Updating STM"
    self._write(text)
    self.create_flash("STM")
    stm_n_ops = self.pk_flash.ihx_n_ops(self.stm_fw.ihx)
    progress_dialog.reset(stm_n_ops, True)
    progress_dialog.title = text
    GUI.invoke_later(progress_dialog.open)
    self.pk_flash.write_ihx(self.stm_fw.ihx, self.stream, mod_print=0x80, \
                            elapsed_ops_cb = progress_dialog.progress)
    self.stop_flash()
    self._write("")

    # Flash NAP.
    text = "Updating NAP"
    self._write(text)
    self.create_flash("M25")
    nap_n_ops = self.pk_flash.ihx_n_ops(self.nap_fw.ihx)
    progress_dialog.reset(nap_n_ops, True)
    progress_dialog.title = text
    GUI.invoke_later(progress_dialog.open)
    self.pk_flash.write_ihx(self.nap_fw.ihx, self.stream, mod_print=0x80, \
                            elapsed_ops_cb = progress_dialog.progress)
    progress_dialog.close()
    del progress_dialog
    self.stop_flash()
    self._write("")

    # Must tell Piksi to jump to application after updating firmware.
    self.link.send_message(ids.BOOTLOADER_JUMP_TO_APP, '\x00')
    self._write("Firmware updates finished.")
    self._write("")

    self.updating = False

  def create_flash(self, flash_type):

    # Reset device if the application is running to put into bootloader mode.
    self.link.send_message(ids.RESET, '')

    self.pk_boot = bootload.Bootloader(self.link)

    self._write("Waiting for bootloader handshake message from Piksi ...")
    reset_prompt = None
    handshake_received = self.pk_boot.wait_for_handshake(1)

    # Prompt user to reset Piksi if we don't receive the handshake message
    # within a reasonable amount of tiime (firmware might be corrupted).
    while not handshake_received:
      reset_prompt = \
        prompt.CallbackPrompt(
                              title="Please Reset Piksi",
                              actions=[prompt.close_button],
                             )

      reset_prompt.text = \
            "You must press the reset button on your Piksi in order\n" + \
            "to update your firmware.\n\n" + \
            "Please press it now.\n\n"

      reset_prompt.run(block=False)

      while not reset_prompt.closed and not handshake_received:
        handshake_received = self.pk_boot.wait_for_handshake(1)

      reset_prompt.kill()
      reset_prompt.wait()

    self.pk_boot.reply_handshake()
    self._write("received bootloader handshake message.")
    self._write("Piksi Onboard Bootloader Version: " + self.pk_boot.version)

    self.pk_flash = flash.Flash(self.link, flash_type)

  def stop_flash(self):
    self.pk_flash.stop()
    self.pk_boot.stop()

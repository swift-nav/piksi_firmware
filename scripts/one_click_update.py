from urllib2 import urlopen, URLError
from json import load as jsonload
import time
from intelhex import IntelHex
import sys
from subprocess import check_output

from threading import Thread

from traits.api import HasTraits, String, Button, Int, Instance
from traitsui.api import View, Handler, Action, Item, InstanceEditor
from pyface.api import GUI

from output_stream import OutputStream

import bootload
import sbp_piksi as ids
import flash

# TODO: handle case where NAP and STM firmwares are bad?
# TODO: sort out handler calling firmware update function

# Not using --dirty so local changes (which could be to non-console files)
# don't make one_click_update think console is out of date.
CONSOLE_VERSION = filter(lambda x: x!='\n', \
                         check_output(['git describe'], shell=True))
INDEX_URL = 'http://download.swift-nav.com/index.json'

class OneClickUpdateHandler(Handler):

  def close(self, info, is_ok): # X button was pressed.
    info.object.handler_executed = True
    return True

  def fw_update_handler(self, info):
    info.ui.dispose()
    info.object.manage_fw_update()
    info.object.handler_executed = True

  def no_fw_update_handler(self, info):
    info.ui.dispose()
    info.object.handler_executed = True

class OneClickUpdateWindow(HasTraits):

  handler = OneClickUpdateHandler()
  output_stream = Instance(OutputStream)
  handler_executed = False
  yes_button = Action(name = "Yes", action = "fw_update_handler", \
                    show_label=False)
  no_button = Action(name = "No", action = "no_fw_update_handler", \
                   show_label=False)

  view = View(
              Item(
                'output_stream',
                style='custom',
                editor=InstanceEditor(),
                height=0.3,
                show_label=False
              ),
              buttons=[yes_button, no_button],
              title="New Piksi Firmware Available",
              handler=OneClickUpdateHandler(),
              height=250,
              width=450,
              resizable=True
             )

  def __init__(self, manage_fw_update):
    self.output_stream = OutputStream()
    self.manage_fw_update = manage_fw_update

  def init_prompt_text(self, local_stm, local_nap, remote_stm, remote_nap):
    init_strings = "Your Piksi STM Firmware Version :\n\t%s\n" % local_stm + \
                   "Newest Piksi STM Firmware Version :\n\t%s\n\n" % remote_stm + \
                   "Your Piksi SwiftNAP Firmware Version :\n\t%s\n" % local_nap + \
                   "Newest Piksi SwiftNAP Firmware Version :\n\t%s\n\n" % remote_nap + \
                   "Upgrade Now?"
    self.output_stream.write(init_strings.encode('ascii', 'ignore'))

class ConsoleOutdatedWindow(HasTraits):
  output_stream = Instance(OutputStream)
  view = View(
              Item(
                'output_stream',
                style='custom',
                editor=InstanceEditor(),
                height=0.3,
                show_label=False
              ),
              buttons=['OK'],
              title="Your Piksi Console is out of date",
              height=250,
              width=450,
              resizable=True
             )

  def __init__(self):
    self.output_stream = OutputStream()

  def init_prompt_text(self, local_console, remote_console):
    txt = "Your Piksi Console is out of date and may be incompatible with " + \
          "current firmware. We highly recommend upgrading to ensure proper " + \
          "behavior. Please visit Swift Navigation's website to download " + \
          "the most recent version.\n\n" + \
          "Your Piksi Console Version :\n\t" + local_console + \
          "\nNewest Piksi Console Version :\n\t" + \
          remote_console + "\n"
    self.output_stream.write(txt)

# TODO: Better error handling
class OneClickUpdate(Thread):

  index = None
  settings = {}

  def __init__(self, link, output=None):
    super(OneClickUpdate, self).__init__()
    self.link = link
    self.fw_update_prompt = OneClickUpdateWindow(self.manage_firmware_updates)
    self.console_outdated_prompt = ConsoleOutdatedWindow()
    if output:
      self.output = output
    else:
      self.output = self.fw_update_prompt.output_stream

  def point_to_settings(self, settings):
    self.settings = settings

  def write(self, text):
    GUI.invoke_later(self.output.write, text)
    GUI.process_events()

  def wait_for_settings(self):
    # TODO: timeout if settings.version info isn't available after some point.
    while True:
      try:
        self.piksi_stm_version = \
            self.settings['system_info']['firmware_version'].value
        self.piksi_nap_version = \
            self.settings['system_info']['nap_version'].value
        break
      except KeyError:
        self.write("Piksi system info not received yet\n")
        time.sleep(1)

  def run(self):

    # Get index that contains file URLs and latest
    # version strings from Swift Nav's website.
    try:
      f = urlopen(INDEX_URL)
      self.index = jsonload(f)
      f.close()
    except URLError:
      self.write("\nError: Failed to download latest file index from Swift Navigation's website (%s). Please visit our website to check that you're running the latest Piksi firmware and Piksi console.\n\n" % INDEX_URL)
      return

    # Wait until console has received Piksi's settings,
    # which contain version strings.
    self.wait_for_settings()

    # Firmware is outdated if version string from Piksi doesn't match
    # latest from website.
    self.stm_fw_outdated = self.index['piksi_v2.3.1']['stm_fw']['version'] \
                               !=  self.piksi_stm_version
    self.nap_fw_outdated = self.index['piksi_v2.3.1']['nap_fw']['version'] \
                               !=  self.piksi_nap_version
    self.fw_update_prompt.init_prompt_text(self.piksi_stm_version, \
                            self.piksi_nap_version, \
                            self.index['piksi_v2.3.1']['stm_fw']['version'], \
                            self.index['piksi_v2.3.1']['nap_fw']['version'])

    # Get firmware files from Swift Nav's website.
    self.nap_ihx = None
    if self.nap_fw_outdated:
      # TODO: timeout?
      try:
        f = urlopen(self.index['piksi_v2.3.1']['nap_fw']['url'])
        self.nap_ihx = IntelHex(f)
        f.close()
      except URLError:
        self.write("\nError: Failed to download latest Piksi SwiftNAP firmware from Swift Navigation's website (%s). Please visit our website to check that you're running the latest firmware.\n" % self.index['piksi_v2.3.1']['nap_fw']['url'])
    self.stm_ihx = None
    if self.stm_fw_outdated:
      # TODO: timeout?
      try:
        f = urlopen(self.index['piksi_v2.3.1']['stm_fw']['url'])
        self.stm_ihx = IntelHex(f)
        f.close()
      except URLError:
        self.write("\nError: Failed to download latest Piksi STM firmware from Swift Navigation's website (%s). Please visit our website to check that you're running the latest firmware.\n" % self.index['piksi_v2.3.1']['stm_fw']['url'])

    # Prompt user to update firmware(s). Only update if firmware was
    # successfully downloaded. If both are out of date, only allow update if we
    # successfully downloaded both files.
    if (self.stm_fw_outdated and self.stm_ihx and not self.nap_fw_outdated) or \
        (self.nap_fw_outdated and self.nap_ihx and not self.stm_fw_outdated) or \
          (self.stm_fw_outdated and self.stm_ihx and \
            self.nap_fw_outdated and self.nap_ihx):
      GUI.invoke_later(self.fw_update_prompt.edit_traits) # Start prompt.
      while not self.fw_update_prompt.handler_executed:
        time.sleep(0.5)

    # Check if console is out of date and notify user if so.
    if (self.index['piksi_v2.3.1']['console']['version'] != CONSOLE_VERSION):
      self.console_outdated_prompt.init_prompt_text(CONSOLE_VERSION, \
                    self.index['piksi_v2.3.1']['console']['version'])
      GUI.invoke_later(self.console_outdated_prompt.edit_traits)

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
    piksi_flash.write_ihx(ihx, self.output, mod_print = 0x1F)

    self.write("\n")


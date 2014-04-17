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
    info.object.manage_fw_update()
    info.object.handler_executed = True
    time.sleep(10)
    info.ui.dispose()

  def no_fw_update_handler(self, info):
    info.object.handler_executed = True
    info.ui.dispose()

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
              title="New Firmware Available",
              handler=OneClickUpdateHandler(),
              height=250,
              width=400,
              resizable=True
             )

  def __init__(self, manage_fw_update):
    self.output_stream = OutputStream()
    self.manage_fw_update = manage_fw_update

  def init_prompt_text(self, local_stm, local_nap, remote_stm, remote_nap):
    init_strings = "Your STM Firmware Version :\n\t%s\n" % local_stm + \
                   "Newest STM Firmware Version :\n\t%s\n\n" % remote_stm + \
                   "Your NAP Firmware Version :\n\t%s\n" % local_nap + \
                   "Newest NAP Firmware Version :\n\t%s\n\n" % remote_nap + \
                   "Upgrade Now?"
    self.output_stream.write(init_strings.encode('ascii', 'ignore'))

#class ConsoleOutdatedWindow(HasTraits):

# TODO: Better error handling
# TODO: Have console output prints
# TODO: Check if network connection is available?
#       Will urlopen just throw URLError?
class OneClickUpdate(Thread):

  index = None

  def __init__(self, link, settings, output=None):
    super(OneClickUpdate, self).__init__()
    self.link = link
    self.settings = settings # Reference to SettingsView.settings dict.
    self.index = None
    self.window = OneClickUpdateWindow(self.manage_firmware_updates)
    if output:
      self.output = output
    else:
      self.output = self.window.output_stream

  def write(self, text):
    GUI.invoke_later(self.output.write, text)
    GUI.process_events()

  # Get index of files from Swift Nav's website.
  def get_file_index(self):
    try:
      f = urlopen(INDEX_URL)
      self.index = jsonload(f)
      f.close()
    except URLError:
      self.write("Error: Unable to retrieve file index from Swift Navigation's website\n")

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

    # Check if firmwares are up to date, and if so update.
    self.get_file_index()

    self.wait_for_settings()

    self.stm_fw_outdated = self.index['piksi_v2.3.1']['stm_fw']['version'] \
                               !=  self.piksi_stm_version
    self.nap_fw_outdated = self.index['piksi_v2.3.1']['nap_fw']['version'] \
                               !=  self.piksi_nap_version
    self.window.init_prompt_text(self.piksi_stm_version, \
                            self.piksi_nap_version, \
                            self.index['piksi_v2.3.1']['stm_fw']['version'], \
                            self.index['piksi_v2.3.1']['nap_fw']['version'])

    if self.stm_fw_outdated or self.nap_fw_outdated:
      GUI.invoke_later(self.window.edit_traits) # Start prompt.
      while not self.window.handler_executed:
        time.sleep(0.5)

    # Check if console is out of date and notify user if so.
    # TODO: add pop up window to tell user console is out of date.
    if (self.index['piksi_v2.3.1']['console']['version'] != CONSOLE_VERSION):
      txt = "Console is out of date and may be incompatible with current\n" + \
            "firmware. We highly recommend upgrading to ensure proper\n" + \
            "behavior. Please visit Swift Navigation's website or Github\n" + \
            "page to upgrade to a newer version.\n" + \
            "Your Console Version  :\n\t" + CONSOLE_VERSION + \
            "\nNewest Console Version :\n\t" + \
            self.index['piksi_v2.3.1']['console']['version'] + "\n"
      self.write(txt)

  def manage_firmware_updates(self):

    self.write("Got NAP fw\n")

    # Get firmware files from Swift Nav's website.
    if self.nap_fw_outdated:
      try:
#        f = urlopen(self.index['piksi_v2.3.1']['nap_fw']['url'])
#        nap_ihx = IntelHex(f)
#        f.close()
        nap_ihx = None
      except URLError:
        self.write("Error: Failed to download NAP firmware from Swift Nav website\n")
        return
    self.write("Got NAP fw\n")
    if self.stm_fw_outdated:
      try:
#        f = urlopen(self.index['piksi_v2.3.1']['stm_fw']['url'])
#        stm_ihx = IntelHex(f)
#        f.close()
        stm_ihx = None
      except URLError:
        self.write("Error: Failed to download STM firmware from Swift Nav website\n")
        return
    self.write("Got STM fw\n")

    # Flash NAP if outdated _AND_ we have STM firmware if it needs to be updated
    if self.nap_fw_outdated:
      self.update_firmware(nap_ihx, "M25")

    #TODO : Change to STM case
    # Flash STM if outdated _AND_ we flashed NAP if it needed to be updated.
    if self.stm_fw_outdated:
      self.update_firmware(nap_ihx, "M25")
      #update_firmware(stm_ihx, self.link, "STM")

    # Piksi needs to jump to application if we updated either firmware.
    if self.nap_fw_outdated or self.stm_fw_outdated:
      self.link.send_message(ids.BOOTLOADER_JUMP_TO_APP, '\x00')

    self.write("Firmware update finished.\n")

  def update_firmware(self, ihx, flash_type):

    # Reset device if the application is running to put into bootloader mode.
    self.link.send_message(ids.RESET, '')

    piksi_bootloader = bootload.Bootloader(self.link)
    self.write("Waiting for bootloader handshake message from Piksi ...\n")
    piksi_bootloader.wait_for_handshake()
    piksi_bootloader.reply_handshake()
    self.write("received bootloader handshake message.\n\n")
    self.write("Piksi Onboard Bootloader Version: " + piksi_bootloader.version + "\n\n")

    piksi_flash = flash.Flash(self.link, flash_type)
#    piksi_flash.write_ihx(ihx, sys.stdout) #TODO: change to embedded window stream


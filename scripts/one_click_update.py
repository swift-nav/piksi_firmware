from urllib2 import urlopen, URLError
from json import load as jsonload
import time
from intelhex import IntelHex
import sys
from subprocess import check_output

from threading import Thread

from traits.api import HasTraits, String, Button, Int, Str, Instance
from traitsui.api import View, Handler, Action, VGroup, HGroup, HSplit, VSplit, Item, InstanceEditor, UItem, TextEditor
from pyface.api import GUI

from output_stream import OutputStream

import bootload
import sbp_piksi as ids
import flash

# Not using --dirty so local changes (which could be to non-console files)
# don't make one_click_update think console is out of date.
CONSOLE_VERSION = filter(lambda x: x!='\n', \
                         check_output(['git describe'], shell=True))
INDEX_URL = 'http://download.swift-nav.com/index.json'
TIMEOUT = 30

class OneClickUpdateHandler(Handler):

  def close(self, info, is_ok): # X button was pressed.
    info.object.update_desired = False
    info.object.handler_executed = True
    return True

  def set_update_desired_true(self, info):
    info.object.update_desired = True
    info.object.handler_executed = True
    info.ui.dispose()

  def set_update_desired_false(self, info):
    info.object.update_desired = False
    info.object.handler_executed = True
    info.ui.dispose()

class OneClickUpdateWindow(HasTraits):

  handler = OneClickUpdateHandler()
  update_desired = False
#  user_prompt_text = Str(None)
  user_prompt_text = Instance(OutputStream)
#  user_prompt_text = List(Str)
  handler_executed = False
  yes_button = Action(name = "Yes", action = "set_update_desired_true")
  no_button = Action(name = "No", action = "set_update_desired_false")

#  view = View('user_prompt_text',
  view = View(
#              UItem('user_prompt_text', show_label=False, resizable=True),
#              View(Item('user_prompt_text',)),# editor=TextEditor(multi_line=True))),
#                         resizable=True,
#                        ),
#                         show_label=False,
#                   handler=_OutputStreamViewHandler()
#              ),
#              Item('user_prompt_text', show_label=False, resizable=True,
#                   editor=InstanceEditor()),
              Item(
                'user_prompt_text',
                style='custom',
                editor=InstanceEditor(),
                height=0.3,
                show_label=False,
              ),
              buttons=[yes_button, no_button],
              title="New Firmware Available",
              handler=OneClickUpdateHandler(),
              height=250,
              width=400,
              resizable=True
             )

  def __init__(self):
    self.user_prompt_text = OutputStream()

  def init_prompt_text(self, local_stm, local_nap, remote_stm, remote_nap):
    init_strings = "Your STM Firmware Version :\n\t%s\n" % local_stm + \
                   "Newest STM Firmware Version :\n\t%s\n\n" % remote_stm + \
                   "Your NAP Firmware Version :\n\t%s\n" % local_nap + \
                   "Newest NAP Firmware Version :\n\t%s\n\n" % remote_nap + \
                   "Upgrade Now?"
    # Hack to get more spaces to print.
    for i in init_strings:
      self.user_prompt_text.write(i.encode('ascii', 'ignore'))

# TODO: Better error handling
# TODO: Have console output prints
# TODO: Check if network connection is available?
#       Will urlopen just throw URLError?
class OneClickUpdate(Thread):

  window = OneClickUpdateWindow()
  index = None

  def __init__(self, link, settings):
    super(OneClickUpdate, self).__init__()
    self.link = link
    self.settings = settings # Reference to SettingsView.settings dict.
    self.index = None

  # Get index of files from Swift Nav's website.
  def get_file_index(self):
    try:
      f = urlopen(INDEX_URL)
      self.index = jsonload(f)
      f.close()
    except URLError:
      print "Error: Unable to retrieve file index from Swift Navigation's website"

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
        print "Piksi system info not received yet"
        time.sleep(1)

  def check_firmware(self):
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
      GUI.invoke_later(self.window.edit_traits) # Start firmware update prompt.
      while not self.window.handler_executed:
#        GUI.invoke_later(self.window.update_user_prompt_text, self.piksi_stm_version, \
#                            self.piksi_nap_version, \
#                            self.index['piksi_v2.3.1']['stm_fw']['version'], \
#                            self.index['piksi_v2.3.1']['nap_fw']['version'])
        time.sleep(0.1)
      print "GUI INVOKED LATER!!!"
      print self.window.handler_executed, type(self.window.handler_executed)
      print self.window.update_desired, type(self.window.update_desired)

  def run(self):

    self.check_firmware()

    # Get firmware files from Swift Nav's website.
    nap_ihx = None
    stm_fw = None
    if self.nap_fw_outdated:
      try:
        f = urlopen(self.index['piksi_v2.3.1']['nap_fw']['url'])
        nap_ihx = IntelHex(f)
        f.close()
      except URLError:
        print "Error: Failed to download NAP firmware from Swift Nav website"
    if self.stm_fw_outdated:
      try:
        f = urlopen(self.index['piksi_v2.3.1']['stm_fw']['url'])
        stm_ihx = IntelHex(f)
        f.close()
      except URLError:
        print "Error: Failed to download STM firmware from Swift Nav website"

    # Flash NAP if outdated _AND_ we have STM firmware if it needs to be updated
    if self.nap_fw_outdated and nap_ihx and not(self.stm_fw_outdated and not stm_ihx):
      print "NAP firmware out of date"
      print "Local Version  : " + piksi_nap_version
      print "Latest Version : " + self.index['piksi_v2.3.1']['nap_fw']['version']
      update_firmware(nap_ihx, self.link, "M25")

    #TODO : Change to STM case
    # Flash STM if outdated _AND_ we flashed NAP if it needed to be updated.
    if self.stm_fw_outdated and stm_ihx and not(self.nap_fw_outdated and not nap_ihx):
      print "STM firmware out of date"
      print "Local Version  : " + piksi_stm_version
      print "Latest Version : " + self.index['piksi_v2.3.1']['stm_fw']['version']
      update_firmware(nap_ihx, self.link, "M25")
      #update_firmware(stm_ihx, self.link, "STM")

    # Piksi needs to jump to application if we updated either firmware.
    if self.nap_fw_outdated or self.stm_fw_outdated:
      self.link.send_message(ids.BOOTLOADER_JUMP_TO_APP, '\x00')

    # Check if console is out of date and notify user if so.
    # TODO: add pop up window to tell user console is out of date.
    if (self.index['piksi_v2.3.1']['console']['version'] != CONSOLE_VERSION):
      print "Console is out of date and may be incompatible with current\n" + \
            "firmware. We highly recommend upgrading to ensure proper\n" + \
            "behavior. Please visit Swift Navigation's website or Github\n" + \
            "page to upgrade to a newer version."
      print "Local Version  : " + CONSOLE_VERSION
      print "Latest Version : " + self.index['piksi_v2.3.1']['console']['version']

def update_firmware(ihx, link, flash_type):

  # Reset device if the application is running to put into bootloader mode.
  link.send_message(ids.RESET, '')

  piksi_bootloader = bootload.Bootloader(link)
  print "Waiting for bootloader handshake message from Piksi ...",
  sys.stdout.flush()
  piksi_bootloader.wait_for_handshake()
  piksi_bootloader.reply_handshake()
  print "received."
  print "Piksi Onboard Bootloader Version:", piksi_bootloader.version

  piksi_flash = flash.Flash(link, flash_type)
#  piksi_flash.write_ihx(ihx)


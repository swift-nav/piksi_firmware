from urllib2 import urlopen, URLError
from json import load as jsonload
import time
from intelhex import IntelHex
import sys

from traits.api import HasTraits, Bool
from traitsui.api import View, Handler

import bootload
import sbp_piksi as ids
import flash

TIMEOUT = 30

class OneClickUpdateHandler(Handler):

  def close(self, info, is_ok):
    if is_ok:
      info.object.update_desired = True
    if not is_ok:
      info.object.update_desired = False
    return True

class OneClickUpdatePopup(HasTraits):

  update_desired = Bool(False)
#  text = 'New firmware is available. Press Okay to update.'
  handler = OneClickUpdateHandler()

  def __init__(self, text):
    self.text = text

  view = View('text',
               title="New Firmware Available",
               handler=OneClickUpdateHandler(),
               buttons = ['OK', 'Cancel'],
               height = 200,
               width = 300)

# TODO: Better error handling
# TODO: Have console output prints
# TODO: Check if network connection is available?
#       Will urlopen just throw URLError?

def main(console, settings):

  # Get index of files from Swift Nav's website.
  try:
    f = urlopen('http://download.swift-nav.com/index.json')
    index = jsonload(f)
    f.close()
  except URLError:
    print "Error: Unable to retrieve file index from Swift Navigation's website"
    return

  # Wait until we have received firmware version from Piksi.
  # TODO: send message to retrieve settings after a period of time
  while True:
    try:
      piksi_stm_version = settings['system_info']['firmware_version'].value
      piksi_nap_version = settings['system_info']['nap_version'].value
      break
    except KeyError:
      print "Piksi system info not received yet"
      time.sleep(1)

  stm_fw_outdated = index['piksi_v2.3.1']['stm_fw']['version'] \
                      !=  piksi_stm_version
  nap_fw_outdated = index['piksi_v2.3.1']['nap_fw']['version'] \
                      !=  piksi_nap_version

#  if stm_fw_outdated or nap_fw_outdated:
  update_prompt_user = OneClickUpdatePopup('New firmware is available. Press Okay to update.')
  update_prompt_user.configure_traits()
  print "Update Desired =", update_prompt_user.update_desired

  # Get firmware files from Swift Nav's website.
  nap_ihx = None
  stm_fw = None
  if nap_fw_outdated:
    try:
      f = urlopen(index['piksi_v2.3.1']['nap_fw']['url'])
      nap_ihx = IntelHex(f)
      f.close()
    except URLError:
      print "Error: Failed to download NAP firmware from Swift Nav website"
  if stm_fw_outdated:
    try:
      f = urlopen(index['piksi_v2.3.1']['stm_fw']['url'])
      stm_ihx = IntelHex(f)
      f.close()
    except URLError:
      print "Error: Failed to download STM firmware from Swift Nav website"

  # Flash NAP if outdated _AND_ we have STM firmware if it needs to be updated
  if nap_fw_outdated and nap_ihx and not(stm_fw_outdated and not stm_ihx):
    print "NAP firmware out of date"
    print "Local Version  : " + piksi_nap_version
    print "Latest Version : " + index['piksi_v2.3.1']['nap_fw']['version']
    update_firmware(nap_ihx, console.link, "M25")

  #TODO : Change to STM case
  # Flash STM if outdated _AND_ we flashed NAP if it needed to be updated.
  if stm_fw_outdated and stm_ihx and not(nap_fw_outdated and not nap_ihx):
    print "STM firmware out of date"
    print "Local Version  : " + piksi_stm_version
    print "Latest Version : " + index['piksi_v2.3.1']['stm_fw']['version']
    update_firmware(nap_ihx, console.link, "M25")
    #update_firmware(stm_ihx, console.link, "STM")

  # Piksi needs to jump to application if we updated either firmware.
  if nap_fw_outdated or stm_fw_outdated:
    console.link.send_message(ids.BOOTLOADER_JUMP_TO_APP, '\x00')

  # Check if console is out of date and notify user if so.
  # TODO: add pop up window to tell user console is out of date.
  if (index['piksi_v2.3.1']['console']['version'] != console.version):
    print "Console is out of date and may be incompatible with current\n" + \
          "firmware. We highly recommend upgrading to ensure proper\n" + \
          "behavior. Please visit Swift Navigation's website or Github\n" + \
          "page to upgrade to a newer version."
    print "Local Version  : " + console.version
    print "Latest Version : " + index['piksi_v2.3.1']['console']['version']

def update_firmware(ihx, link, flash_type):

  # Reset device if the application is running.
  link.send_message(ids.RESET, '')

  piksi_bootloader = bootload.Bootloader(link)
  print "Waiting for bootloader handshake message from Piksi ...",
  sys.stdout.flush()
  piksi_bootloader.wait_for_handshake()
  piksi_bootloader.reply_handshake()
  print "received."
  print "Piksi Onboard Bootloader Version:", piksi_bootloader.version

  piksi_flash = flash.Flash(link, flash_type)
  piksi_flash.write_ihx(ihx)

if __name__ == "__main__":
  main()

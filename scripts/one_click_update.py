from urllib2 import urlopen, URLError
from json import load as jsonload
import time
from intelhex import IntelHex
import sys

import bootload
import sbp_piksi as ids
import flash

TIMEOUT = 30

# TODO: Better error handling
# TODO: Have console output prints

def main(console, settings):

  print "Version =", console.version

  # Get index of files from Swift Nav's website.
  try:
    f = urlopen('http://download.swift-nav.com/index.json', timeout=TIMEOUT)
    index = jsonload(f)
    f.close()
  except URLError:
    print "Error: Unable to retrieve firmware index from Swift Navigation's website"
    return

  # Wait until we have received firmware version from Piksi
  # TODO: send settings message to retrieve settings
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

  #TODO : add timeout, properly execute STM case.

  if nap_fw_outdated:
    print "NAP firmware out of date"
    print "Local : " + piksi_nap_version
    print "Remote : " + index['piksi_v2.3.1']['nap_fw']['version']

    # Get latest firmware from Swift Nav's website
    try:
      f = urlopen(index['piksi_v2.3.1']['nap_fw']['url'], timeout=TIMEOUT)
      hexfile = IntelHex(f)
      f.close()
    except URLError:
      print "Error: Unable to retrieve SwiftNAP firmware from Swift Nav's website"
      time.sleep(1)

    # Reset device if the application is running.
    console.link.send_message(ids.RESET, '')

    piksi_bootloader = bootload.Bootloader(console.link)
    print "Waiting for bootloader handshake message from Piksi ...",
    sys.stdout.flush()
    piksi_bootloader.wait_for_handshake()
    piksi_bootloader.reply_handshake()
    print "received."
    print "Piksi Onboard Bootloader Version:", piksi_bootloader.version

    piksi_flash = flash.Flash(console.link, flash_type="M25")
#    piksi_flash.write_ihx(hexfile)
    # Deleting Flash/Bootloader removes SerialLink callbacks
    # for these objects. It's a good idea to do this before creating new
    # Flash/Bootloader objects that will use the same SerialLink and register
    # new callback methods with the same message IDs.
    del piksi_flash
    del piksi_bootloader

  #TODO : properly execute STM case.
  if stm_fw_outdated:
    print "STM firmware out of date"
    print "Local : " + piksi_stm_version
    print "Remote : " + index['piksi_v2.3.1']['stm_fw']['version']

    # Get latest firmware from Swift Nav's website
    try:
      f = urlopen(index['piksi_v2.3.1']['nap_fw']['url'], timeout=TIMEOUT)
      hexfile = IntelHex(f)
      f.close()
    except URLError:
      print "Error: Unable to retrieve STM firmware from Swift Nav's website"
      time.sleep(1)

    # Reset device if the application is running.
    console.link.send_message(ids.RESET, '')

    piksi_bootloader = bootload.Bootloader(console.link)
    print "Waiting for bootloader handshake message from Piksi ...",
    sys.stdout.flush()
    piksi_bootloader.wait_for_handshake()
    piksi_bootloader.reply_handshake()
    print "received."
    print "Piksi Onboard Bootloader Version:", piksi_bootloader.version

    piksi_flash = flash.Flash(console.link, flash_type="M25")
#    piksi_flash.write_ihx(hexfile)

    del piksi_flash
    del piksi_bootloader

  if (index['piksi_v2.3.1']['console']['version'] != console.version):
    print "Console is out of date. Please visit Swift Navigation's website", \
          "or Github page to upgrade to a newer version."
    print "Local : " + console.version
    print "Remote : " + index['piksi_v2.3.1']['console']['version']

  # Need to jump out of the bootloader if we updated either firmware.
  if nap_fw_outdated or stm_fw_outdated:
    console.link.send_message(ids.BOOTLOADER_JUMP_TO_APP, '\x00')

if __name__ == "__main__":
  main()

#!/usr/bin/env python
# Copyright (C) 2011-2014 Swift Navigation Inc.
# Contact: Niels Joubert <njoubert@gmail.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

#This script resets the FDTI settings to its default values

import time
import struct
import argparse
import sys

from sbp.piksi import SBP_MSG_SETTINGS, SBP_MSG_SETTINGS_SAVE
from sbp.client.main import *

def send_setting(link, section, name, value):
  link.send(SBP_MSG_SETTINGS, '%s\0%s\0%s\0' % (section, name, value))


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Piksi Recover FTDI')
  parser.add_argument('-p', '--port',
                      default=[SERIAL_PORT], nargs=1,
                      help='specify the serial port to use.')
  parser.add_argument("-b", "--baud",
                      default=[SERIAL_BAUD], nargs=1,
                      help="specify the baud rate to use.")
  parser.add_argument("-f", "--ftdi",
                      help="use pylibftdi instead of pyserial.",
                      action="store_true")
  args = parser.parse_args()
  serial_port = args.port[0]
  baud = args.baud[0]
  with get_driver(args.ftdi, serial_port, baud) as driver:
    with Handler(driver.read, driver.write) as link:

      print "Resetting mask to 0xff"
      send_setting(link, "uart_ftdi", "sbp_message_mask", "65535")
      time.sleep(0.5)

      print "Resetting baudrate to 1Mbps"
      send_setting(link, "uart_ftdi", "baudrate", "1000000")
      time.sleep(0.5)

      print "Resetting mode to SBP"
      send_setting(link, "uart_ftdi", "mode", "SBP")
      time.sleep(0.5)

      print "Attempting to save settings"
      link.send(SBP_MSG_SETTINGS_SAVE, "")
      time.sleep(0.5)
      link.send(SBP_MSG_SETTINGS_SAVE, "")
      time.sleep(0.5)
      link.send(SBP_MSG_SETTINGS_SAVE, "")
      time.sleep(0.5)
      link.send(SBP_MSG_SETTINGS_SAVE, "")

      print "Sent Settings Reset message to return FTDI to defaults"

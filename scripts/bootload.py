#!/usr/bin/env python
#
# Bootloader for the Swift Navigation Piksi GPS Receiver
#
# Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
# Copyright (C) 2011 Piotr Esden-Tempski <piotr@esden.net>
# Copyright (C) 2013-2014 Swift Navigation Inc <www.swift-nav.com>
#
# Contacts: Colin Beighley <colin@swift-nav.com>
#           Fergus Noble <fergus@swift-nav.com>
#
# Based on luftboot, a bootloader for the Paparazzi UAV project.
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

import time
import struct

from sbp.piksi import *

class Bootloader():

  def __init__(self, link):
    self.stopped = False
    self.handshake_received = False
    self.version = None
    self.link = link
    self.link.add_callback(self._handshake_callback, SBP_MSG_BOOTLOADER_HANDSHAKE)

  def __del__(self):
    if not self.stopped:
      self.stop()

  def stop(self):
    self.stopped = True
    self.link.remove_callback(self._handshake_callback, SBP_MSG_BOOTLOADER_HANDSHAKE)

  def _handshake_callback(self, sbp_msg):
    if len(sbp_msg.payload)==1 and struct.unpack('B', sbp_msg.payload[0])==0:
      # == v0.1 of the bootloader, returns hardcoded version number 0.
      self.version = "v0.1"
    else:
      # > v0.1 of the bootloader, returns git commit string.
      self.version = sbp_msg.payload[:]
    self.handshake_received = True

  def wait_for_handshake(self, timeout=None):
    if timeout is not None:
      t0 = time.time()
    self.handshake_received = False
    while not self.handshake_received:
      time.sleep(0.1)
      if timeout is not None:
        if time.time()-timeout > t0:
          return False
    return True

  def reply_handshake(self):
    self.link.send(SBP_MSG_BOOTLOADER_HANDSHAKE, '\x00')

  def jump_to_app(self):
    self.link.send(SBP_MSG_BOOTLOADER_JUMP_TO_APP, '\x00')

if __name__ == "__main__":
  import argparse
  import thread
  import sys
  from intelhex import IntelHex
  import flash
  parser = argparse.ArgumentParser(description='Piksi Bootloader')
  parser.add_argument("file",
                      help="the Intel hex file to write to flash.")
  parser.add_argument('-m', '--m25',
                      help='write the file to the M25 (FPGA) flash.',
                      action="store_true")
  parser.add_argument('-s', '--stm',
                      help='write the file to the STM flash.',
                      action="store_true")
  parser.add_argument('-e', '--erase',
                      help='erase sectors 1-11 of the STM flash.',
                      action="store_true")
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
  if args.stm and args.m25:
    parser.error("Only one of -s or -m options may be chosen")
    sys.exit(2)
  elif not args.stm and not args.m25:
    parser.error("One of -s or -m options must be chosen")
    sys.exit(2)
  if args.erase and not args.stm:
    parser.error("The -e option requires the -s option to also be chosen")
  ihx = IntelHex(args.file)

  with get_driver(args.ftdi, serial_port, baud) as driver:
    with Handler(driver.read, driver.write) as link:
      link.add_callback(printer, SBP_MSG_PRINT)
      link.start()

      # Reset device if the payload is running.
      link.send(SBP_MSG_RESET, '')
      time.sleep(0.2)

      # Tell Bootloader we want to write to the flash.
      piksi_bootloader = Bootloader(link)
      print "Waiting for bootloader handshake message from Piksi ...",
      sys.stdout.flush()
      try:
        piksi_bootloader.wait_for_handshake()
      except KeyboardInterrupt:
        # Clean up and exit.
        piksi_bootloader.stop()

      piksi_bootloader.reply_handshake()
      print "received."
      print "Piksi Onboard Bootloader Version:", piksi_bootloader.version

      # Catch all other errors and exit cleanly.
      try:
        if args.stm:
          piksi_flash = flash.Flash(link, flash_type="STM")
        elif args.m25:
          piksi_flash = flash.Flash(link, flash_type="M25")

        if args.erase:
          for s in range(1,12):
            print "\rErasing STM Sector", s,
            sys.stdout.flush()
            piksi_flash.erase_sector(s)
          print

        piksi_flash.write_ihx(ihx, sys.stdout, mod_print = 0x10)

        print "Bootloader jumping to application"
        piksi_bootloader.jump_to_app()

        piksi_flash.stop()
        piksi_bootloader.stop()
      except:
        import traceback
        traceback.print_exc()
      finally:
        # Clean up and exit.
        if not piksi_bootloader.stopped:
          piksi_bootloader.stop()
        try:
          if not piksi_flash.stopped:
            piksi_flash.stop()
        except NameError:
          pass


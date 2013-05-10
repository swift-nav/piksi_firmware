#!/usr/bin/env python
#
# Bootloader for the Swift Navigation Piksi GPS Receiver
#
# Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
# Copyright (C) 2011 Piotr Esden-Tempski <piotr@esden.net>
# Copyright (C) 2013 Swift Navigation Inc <www.swift-nav.com>
#
# Contact: Colin Beighley <colin@swift-nav.com>
#
# Based on luftboot, a bootloader for the Paparazzi UAV project.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import serial_link
import struct
import time
import sys
from intelhex import IntelHex
from itertools import groupby

MSG_STM_FLASH_PROGRAM_BYTE = 0xE0
MSG_STM_FLASH_PROGRAM      = 0xE4
MSG_STM_FLASH_READ         = 0xE1 # both C and Python ID
MSG_STM_FLASH_ERASE_SECTOR = 0xE2
MSG_STM_FLASH_COMPLETE     = 0xE0
MSG_BOOTLOADER_HANDSHAKE   = 0xE3
MSG_STM_UNIQUE_ID          = 0xE4

MSG_JUMP_TO_APP = 0xA0

ADDRS_PER_OP = 250

def sector_of_address(addr):
  if   addr >= 0x08000000 and addr < 0x08004000:
    return 0
  elif addr >= 0x08004000 and addr < 0x08008000:
    return 1
  elif addr >= 0x08008000 and addr < 0x0800C000:
    return 2
  elif addr >= 0x0800C000 and addr < 0x08010000:
    return 3
  elif addr >= 0x08010000 and addr < 0x08020000:
    return 4
  elif addr >= 0x08020000 and addr < 0x08040000:
    return 5
  elif addr >= 0x08040000 and addr < 0x08060000:
    return 6
  elif addr >= 0x08060000 and addr < 0x08080000:
    return 7
  elif addr >= 0x08080000 and addr < 0x080A0000:
    return 8
  elif addr >= 0x080A0000 and addr < 0x080C0000:
    return 9
  elif addr >= 0x080C0000 and addr < 0x080E0000:
    return 10
  elif addr >= 0x080E0000 and addr < 0x08100000:
    return 11
  else:
    return None

def roundup_multiple(x, multiple):
  return x if x % multiple == 0 else x + multiple - x % multiple

def rounddown_multiple(x, multiple):
  return x if x % multiple == 0 else x - x % multiple

def ihx_ranges(ihx):
  def first_last(x):
      first = x.next()
      last = first
      for last in x:
          pass
      return (first[1], last[1])
  return [first_last(v) for k, v in
          groupby(enumerate(ihx.addresses()), lambda (i, x) : i - x)]

def sectors_used(ihx):
    ranges = ihx_ranges(ihx)
    sectors = set()
    for s, e in ranges:
        sectors |= set(range(sector_of_address(s), sector_of_address(e)+1))
    return sorted(list(sectors))

# Currently just serves to block until callbacks
# for each command are received
class STM32Flash():
  bootloader_ready = False
  _waiting_for_callback = False
  _read_callback_data = []

  def __init__(self, link):
    self.link = link
    self.link.add_callback(MSG_STM_FLASH_COMPLETE, self._done_callback)
    self.link.add_callback(MSG_STM_FLASH_READ, self._read_callback)
    self.link.add_callback(MSG_BOOTLOADER_HANDSHAKE,
                           self._bootloader_ready_callback)

  def erase_sector(self, sector):
    msg_buf = struct.pack("B", sector)
    self._waiting_for_callback = True
    link.send_message(MSG_STM_FLASH_ERASE_SECTOR, msg_buf)
    while self._waiting_for_callback == True:
      time.sleep(0.01)

  def program_byte(self, address, byte):
    msg_buf = struct.pack("<IB", address, byte)
    self._waiting_for_callback = True
    link.send_message(MSG_STM_FLASH_PROGRAM_BYTE, msg_buf)
    while self._waiting_for_callback == True:
      time.sleep(0.01)

  def program(self, address, data):
    msg_buf = struct.pack("<IB", address, len(data))
    self._waiting_for_callback = True
    link.send_message(MSG_STM_FLASH_PROGRAM, msg_buf + data)
    while self._waiting_for_callback == True:
      time.sleep(0.01)

  def read(self, address, length):
    msg_buf = struct.pack("<IB", address, length)
    self._waiting_for_callback = True
    link.send_message(MSG_STM_FLASH_READ, msg_buf)
    while self._waiting_for_callback == True:
      time.sleep(0.01)
    return self._read_callback_data

  def _done_callback(self, data):
    self._waiting_for_callback = False

  def _read_callback(self, data):
    # 4 bytes addr, 1 byte length, length bytes data
    addr = struct.unpack('<I', data[0:4])[0];
    length = struct.unpack('B', data[4])[0];
    self._read_callback_data = list(struct.unpack(str(length)+'B', data[5:]))
    self._waiting_for_callback = False

  def _bootloader_ready_callback(self, data):
    self.bootloader_ready = True

if __name__ == "__main__":
  import argparse

  parser = argparse.ArgumentParser(description='Piksi Bootloader')
  parser.add_argument("file",
                      help="the sample data file to process")
  parser.add_argument('-p', '--port',
                     default=[serial_link.DEFAULT_PORT], nargs=1,
                     help='specify the serial port to use.')
  parser.add_argument("-f", "--ftdi",
                    help="use pylibftdi instead of pyserial.",
                    action="store_true")
  args = parser.parse_args()
  serial_port = args.port[0]

  print "Waiting for device to be plugged in ...",
  sys.stdout.flush()

  found_device = False
  while not found_device:
    try:
      link = serial_link.SerialLink(serial_port, use_ftdi=args.ftdi)
      found_device = True
    except KeyboardInterrupt:
      # Clean up and exit
      link.close()
      sys.exit()
    except:
      # Couldn't find device
      time.sleep(0.01)

  link.add_callback(serial_link.MSG_PRINT, serial_link.default_print_callback)
  print "link with device successfully created."

  # Create STM32Flash object and pass Piksi serial link to it
  flash = STM32Flash(link)

  # Wait until device informs us that it is ready to receive program
  print "Waiting for device to tell us it is ready to bootload ...",
  sys.stdout.flush()
  try:
    while not flash.bootloader_ready:
      time.sleep(0.1)
  except KeyboardInterrupt:
    # Clean up and exit
    link.close()
    sys.exit()
  print "received handshake signal."

  # Send message to device to let it know we want to change the application
  link.send_message(MSG_BOOTLOADER_HANDSHAKE, '\x00')

  ihx = IntelHex(args.file)
  # Erase sector of Piksi's flash where binary is to go
  # Don't erase sectors 0-3 or you'll erase the bootloader
  for sector in sectors_used(ihx):
    if sector in range(4, 12):
      print "Erasing sector", sector
      flash.erase_sector(sector)
    else:
      print "Sector", sector, "out of bounds, not erasing"

  for start, end in ihx_ranges(ihx):
    for addr in range(start, end, ADDRS_PER_OP):
      print ("Programming flash at 0x%08X\r" % addr),
      sys.stdout.flush()
      binary = ihx.tobinstr(start=addr, size=ADDRS_PER_OP)
      flash.program(addr, binary)
      flash_readback = flash.read(addr, ADDRS_PER_OP)
      if flash_readback != map(ord, binary):
        raise Exception('data read from flash != data written to flash')
  print "\nDone programming flash, telling device to jump to application"

  # Tell STM to jump to application, as programming is finished
  link.send_message(MSG_JUMP_TO_APP, '\x00')

  # Wait for ctrl+C until we exit
  try:
    while(1):
      time.sleep(0.1)
  except KeyboardInterrupt:
    pass

  # Clean up and exit
  link.close()
  sys.exit()


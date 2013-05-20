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

MSG_STM_FLASH_PROGRAM = 0xE0 # Callback in C
MSG_STM_FLASH_READ    = 0xE1 # Callback in both C and Python
MSG_STM_FLASH_ERASE   = 0xE2 # Callback in C
MSG_STM_FLASH_DONE    = 0xE0 # Callback in Python

MSG_STM_UNIQUE_ID = 0xE5 # Callback in both C and Python

MSG_M25_FLASH_WRITE = 0xF0 # Callback in C
MSG_M25_FLASH_READ  = 0xF1 # Callback in both C and Python
MSG_M25_FLASH_ERASE = 0xF2 # Callback in C
MSG_M25_FLASH_DONE  = 0xF0 # Callback in Python

MSG_BOOTLOADER_HANDSHAKE   = 0xC0 # Callback in both C and Python
MSG_BOOTLOADER_JUMP_TO_APP = 0xC1 # Callback in C

ADDRS_PER_OP = 250

def stm_addr_sector_map(addr):
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

def m25_addr_sector_map(addr):
  if addr < 0 or addr > 0xFFFFF:
    raise ValueError
  return addr >> 16

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

class Flash():
  bootloader_ready = False
  _waiting_for_callback = False
  _read_callback_data = []

  def __init__(self, flash_type, link):
    self.link = link
    self.flash_type = flash_type
    if flash_type == "STM":
      self.link.add_callback(MSG_STM_FLASH_DONE, self._done_callback)
      self.link.add_callback(MSG_STM_FLASH_READ, self._read_callback)
      self.link.add_callback(MSG_BOOTLOADER_HANDSHAKE,
                             self._bootloader_ready_callback)
      self.flash_msg_read = MSG_STM_FLASH_READ
      self.flash_msg_erase = MSG_STM_FLASH_ERASE
      self.flash_msg_write = MSG_STM_FLASH_WRITE
      self.addr_sector_map = stm_addr_sector_map
    elif flash_type == "M25":
      self.link.add_callback(MSG_M25_FLASH_DONE, self._done_callback)
      self.link.add_callback(MSG_M25_FLASH_READ, self._read_callback)
      self.link.add_callback(MSG_BOOTLOADER_HANDSHAKE,
                             self._bootloader_ready_callback)
      self.flash_msg_read = MSG_M25_FLASH_READ
      self.flash_msg_erase = MSG_M25_FLASH_ERASE
      self.flash_msg_write = MSG_M25_FLASH_WRITE
      self.addr_sector_map = m25_addr_sector_map
    else:
      raise ValueError
    while not self.bootloader_ready:
      time.sleep(0.01)

  def sectors_used(addrs):
    sectors = set()
    for s, e in addrs:
      sectors |= set(range(self.addr_sector_map(s), self.addr_sector_map(e)+1))
    return sorted(list(sectors))

  def sector_restricted(self,sector):
    if self.flash_type == "STM":
      if sector < 4: # assuming bootloader occupies sectors 0-3
        return True
      else:
        return False
    elif self.flash_type == "M25":
      if sector == 15: # assuming authentication hash occupies sector 15
        return True
      else:
        return False
    return None

  def erase_sector(self, sector):
    if self.sector_restricted(sector):
      raise Exception("Tried to erase restricted sector")
    self._waiting_for_callback = True
    link.send_message(self.flash_msg_erase, msg_buf)
    while self._waiting_for_callback == True:
      time.sleep(0.001)

  def program(self, address, data):
    msg_buf = struct.pack("<IB", address, len(data))
    self._waiting_for_callback = True
    link.send_message(self.flash_msg_write, msg_buf + data)
    while self._waiting_for_callback == True:
      time.sleep(0.001)

  def read(self, address, length):
    msg_buf = struct.pack("<IB", address, length)
    self._waiting_for_callback = True
    link.send_message(self.flash_msg_read, msg_buf)
    while self._waiting_for_callback == True:
      time.sleep(0.001)
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

  print "link with device successfully created."
  link.add_callback(serial_link.MSG_PRINT, serial_link.default_print_callback)

  # Create Flash object and pass Piksi serial link to it
  stm_flash = Flash(link, flash_type="STM")

  # Wait until device informs us that it is ready to receive program
  print "Waiting for device to tell us it is ready to bootload ...",
  sys.stdout.flush()
  try:
    while not stm_flash.bootloader_ready:
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
  addrs = ihx_ranges(ihx)
  for sector in stm_flash.sectors_used(addrs):
    print "Erasing sector", sector
    stm_flash.erase_sector(sector)

  for start, end in ihx_ranges(ihx):
    for addr in range(start, end, ADDRS_PER_OP):
      print ("Programming flash at 0x%08X\r" % addr),
      sys.stdout.flush()
      binary = ihx.tobinstr(start=addr, size=ADDRS_PER_OP)
      stm_flash.program(addr, binary)
      flash_readback = stm_flash.read(addr, ADDRS_PER_OP)
      if flash_readback != map(ord, binary):
        raise Exception('data read from flash != data written to flash')
  print "\nDone programming flash, telling device to jump to application"

  # Tell STM to jump to application, as programming is finished
  link.send_message(MSG_BOOTLOADER_JUMP_TO_APP, '\x00')

  # Wait for ctrl+C until we exit
  try:
    while(1):
      time.sleep(0.1)
  except KeyboardInterrupt:
    pass

  # Clean up and exit
  link.close()
  sys.exit()


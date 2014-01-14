#!/usr/bin/env python
# Copyright (C) 2013 Swift Navigation Inc <www.swift-nav.com>

import serial_link
import sbp_messages as ids
import struct
import time
import sys
from itertools import groupby
import new

ADDRS_PER_OP = 128

M25_SR_SRWD = 1 << 7
M25_SR_BP2  = 1 << 4
M25_SR_BP1  = 1 << 3
M25_SR_BP0  = 1 << 2
M25_SR_WEL  = 1 << 1
M25_SR_WIP  = 1 << 0

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

def ihx_ranges(ihx):
  def first_last(x):
    first = x.next()
    last = first
    for last in x:
      pass
    return (first[1], last[1])
  return [first_last(v) for k, v in
          groupby(enumerate(ihx.addresses()), lambda (i, x) : i - x)]

# Defining separate functions to lock/unlock STM sectors and to read/write M25
# status register, as there isn't a great way to define lock/unlock sector
# callbacks that will be general to both flashes (see M25Pxx datasheet).
# Functions conditionally bound to Flash instance in Flash.__init__(),
# depending on flash_type.

# Lock sector of STM flash (0-11).
def _stm_lock_sector(self, sector):
  self._waiting_for_callback = True
  msg_buf = struct.pack("B", sector)
  self.link.send_message(ids.STM_FLASH_LOCK_SECTOR, msg_buf)
  while self._waiting_for_callback:
    time.sleep(0.001)

# Unlock sector of STM flash (0-11).
def _stm_unlock_sector(self, sector):
  self._waiting_for_callback = True
  msg_buf = struct.pack("B", sector)
  self.link.send_message(ids.STM_FLASH_UNLOCK_SECTOR, msg_buf)
  while self._waiting_for_callback:
    time.sleep(0.001)

# Write M25 status register (8 bits).
def _m25_write_status(self, sr):
  self._waiting_for_callback = True
  msg_buf = struct.pack("B", sr)
  self.link.send_message(ids.M25_FLASH_WRITE_STATUS, msg_buf)
  while self._waiting_for_callback:
    time.sleep(0.001)

# Read M25 status register (8 bits).
def _m25_read_status(self):
  self._waiting_for_callback = True
  self.link.send_message(ids.M25_FLASH_READ_STATUS, '\x00')
  while self._waiting_for_callback:
    time.sleep(0.001)
  return self.status_register

# Callback to receive M25 status register data (8 bits).
def _m25_read_status_callback(self, data):
  self.status_register = struct.unpack('B', data[0])[0]
  self._waiting_for_callback = False

class Flash():

  def __init__(self, link, flash_type):
    self.status = ''
    self.link = link
    self.flash_type = flash_type
    self._waiting_for_callback = False
    self._read_callback_data = []
    self.link.add_callback(ids.FLASH_DONE, self._done_callback)
    self.link.add_callback(ids.FLASH_READ, self._read_callback)
    if self.flash_type == "STM":
      self.flash_type_byte = 0
      self.addr_sector_map = stm_addr_sector_map
      # Add STM-specific functions.
      self.__dict__['lock_sector'] = \
          new.instancemethod(_stm_lock_sector, self, Flash)
      self.__dict__['unlock_sector'] = \
          new.instancemethod(_stm_unlock_sector, self, Flash)
    elif self.flash_type == "M25":
      self.flash_type_byte = 1
      self.addr_sector_map = m25_addr_sector_map
      self.status_register = None
      # Add M25-specific functions.
      self.__dict__['write_status'] = \
          new.instancemethod(_m25_write_status, self, Flash)
      self.__dict__['read_status'] = \
          new.instancemethod(_m25_read_status, self, Flash)
      self.__dict__['_read_status_callback'] = \
          new.instancemethod(_m25_read_status_callback, self, Flash)
      self.link.add_callback(ids.M25_FLASH_READ_STATUS, \
                             self._read_status_callback)
    else:
      raise ValueError

  def __str__(self):
    return self.status

  def sectors_used(self, addrs):
    sectors = set()
    for s, e in addrs:
      sectors |= set(range(self.addr_sector_map(s), self.addr_sector_map(e)+1))
    return sorted(list(sectors))

  def erase_sector(self, sector):
    msg_buf = struct.pack("BB", self.flash_type_byte, sector)
    self._waiting_for_callback = True
    self.link.send_message(ids.FLASH_ERASE, msg_buf)
    while self._waiting_for_callback == True:
      time.sleep(0.001)

  def program(self, address, data):
    msg_buf = struct.pack("B", self.flash_type_byte)
    msg_buf += struct.pack("<I", address)
    msg_buf += struct.pack("B", len(data))
    self._waiting_for_callback = True
    self.link.send_message(ids.FLASH_PROGRAM, msg_buf + data)
    while self._waiting_for_callback == True:
      time.sleep(0.001)

  def read(self, address, length):
    msg_buf = struct.pack("B", self.flash_type_byte)
    msg_buf += struct.pack("<I", address)
    msg_buf += struct.pack("B", length)
    self._waiting_for_callback = True
    self.link.send_message(ids.FLASH_READ, msg_buf)
    while self._waiting_for_callback == True:
      time.sleep(0.001)
    assert address == self._read_callback_address, \
        "Address received (0x%08x) does not match address sent (0x%08x)" % \
        (self._read_callback_address, address)
    assert length == self._read_callback_length, \
        "Length received (0x%08x) does not match length sent (0x%08x)" % \
        (self._read_callback_length, length)
    return self._read_callback_data

  def _done_callback(self, data):
    self._waiting_for_callback = False

  def _read_callback(self, data):
    # 4 bytes addr, 1 byte length, length bytes data
    self._read_callback_address = struct.unpack('<I', data[0:4])[0];
    self._read_callback_length = struct.unpack('B', data[4])[0];
    length = self._read_callback_length
    self._read_callback_data = list(struct.unpack(str(length)+'B', data[5:]))
    self._waiting_for_callback = False

  def write_ihx(self, ihx, verbose=True):
    # Erase sectors
    ihx_addrs = ihx_ranges(ihx)
    for sector in self.sectors_used(ihx_addrs):
      self.status = self.flash_type + " Flash: Erasing sector %d" % sector
      if verbose:
        print '\r' + self.status,
        sys.stdout.flush()
      self.erase_sector(sector)
    if verbose:
      print ''

    # Write data to flash and validate
    start_time = time.time()
    for start, end in ihx_addrs:
      for addr in range(start, end, ADDRS_PER_OP):
        self.status = self.flash_type + " Flash: Programming address" + \
                                        " 0x%08X" % addr
        if verbose:
          print '\r' + self.status,
          sys.stdout.flush()
        binary = ihx.tobinstr(start=addr, size=ADDRS_PER_OP)
        self.program(addr, binary)
        flash_readback = self.read(addr, ADDRS_PER_OP)
        if flash_readback != map(ord, binary):
          raise Exception('Data read from flash != Data written to flash')
    self.status = self.flash_type + " Flash: Successfully programmed and " + \
                                    "verified, total time = %d seconds" % \
                                    int(time.time()-start_time)
    if verbose:
      print '\n' + self.status


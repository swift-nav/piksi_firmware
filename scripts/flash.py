#!/usr/bin/env python
# Copyright (C) 2011-2014 Swift Navigation Inc.
# Contact: Fergus Noble <fergus@swift-nav.com>
#          Colin Beighley <colin@swift-nav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

import serial_link
import struct
import time
import sys
import new
from intelhex import IntelHex
from threading import Lock

from itertools import groupby
from sbp.piksi import *

# 0.5 * ADDRS_PER_OP * MAX_QUEUED_OPS (plus SBP overhead) is how much each of
# the STM32 TX/RX buffers will be filled by the program/read callback messages.
ADDRS_PER_OP = 128
MAX_QUEUED_OPS = 20

M25_SR_SRWD = 1 << 7
M25_SR_BP2  = 1 << 4
M25_SR_BP1  = 1 << 3
M25_SR_BP0  = 1 << 2
M25_SR_WEL  = 1 << 1
M25_SR_WIP  = 1 << 0

STM_RESTRICTED_SECTORS = [0]
STM_N_SECTORS = 12

M25_RESTRICTED_SECTORS = [15]
M25_N_SECTORS = 16

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
    raise IndexError("Attempted to access flash memory at (%s) outside of range (wrong firmware file?)." % hex(addr))
    return None

def m25_addr_sector_map(addr):
  if addr < 0 or addr > 0xFFFFF:
    raise IndexError("Attempted to access flash memory at (%s) outside of range (wrong firmware file?)." % hex(addr))
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

def sectors_used(addrs, addr_sector_map):
  sectors = set()
  for s, e in addrs:
    sectors |= set(range(addr_sector_map(s), addr_sector_map(e)+1))
  return sorted(list(sectors))

# Operations it will take to write a particular hexfile using flash.write_ihx.
def ihx_n_ops(ihx, addr_sector_map):
  ihx_addrs = ihx_ranges(ihx)
  erase_ops = len(sectors_used(ihx_addrs, addr_sector_map))
  program_ops_addr_args = [range(s,e,ADDRS_PER_OP) for s,e in ihx_addrs]
  program_ops = sum([len(l) for l in program_ops_addr_args])
  read_ops = program_ops # One read callback for every program callback.
  return erase_ops + program_ops + read_ops

# Defining separate functions to lock/unlock STM sectors and to read/write M25
# status register, as there isn't a great way to define lock/unlock sector
# callbacks that will be general to both flashes (see M25Pxx datasheet).
# Functions conditionally bound to Flash instance in Flash.__init__(),
# depending on flash_type.

# Lock sector of STM flash (0-11).
def _stm_lock_sector(self, sector):
  msg_buf = struct.pack("B", sector)
  self.inc_n_queued_ops()
  self.link.send_message(SBP_MSG_STM_FLASH_LOCK_SECTOR, msg_buf)
  while self.get_n_queued_ops() > 0:
    time.sleep(0.001)

# Unlock sector of STM flash (0-11).
def _stm_unlock_sector(self, sector):
  msg_buf = struct.pack("B", sector)
  self.inc_n_queued_ops()
  self.link.send_message(SBP_MSG_STM_FLASH_UNLOCK_SECTOR, msg_buf)
  while self.get_n_queued_ops() > 0:
    time.sleep(0.001)

# Write M25 status register (8 bits).
def _m25_write_status(self, sr):
  msg_buf = struct.pack("B", sr)
  self.inc_n_queued_ops()
  self.link.send_message(SBP_MSG_M25_FLASH_WRITE_STATUS, msg_buf)
  while self.get_n_queued_ops() > 0:
    time.sleep(0.001)

class Flash():

  def __init__(self, link, flash_type):
    self._n_queued_ops = 0
    self.nqo_lock = Lock()
    self.stopped = False
    self.status = ''
    self.link = link
    self.flash_type = flash_type
    self._read_callback_ihx = IntelHex()
    self.link.add_callback(SBP_MSG_FLASH_DONE, self._done_callback)
    self.link.add_callback(SBP_MSG_FLASH_READ, self._read_callback)
    self.ihx_elapsed_ops = 0 # N operations finished in self.write_ihx
    self.ihx_total_ops = None # Total operations in self.write_ihx call
    if self.flash_type == "STM":
      self.flash_type_byte = 0
      self.addr_sector_map = stm_addr_sector_map
      # Add STM-specific functions.
      self.__dict__['lock_sector'] = \
          new.instancemethod(_stm_lock_sector, self, Flash)
      self.__dict__['unlock_sector'] = \
          new.instancemethod(_stm_unlock_sector, self, Flash)
      self.n_sectors = STM_N_SECTORS
      self.restricted_sectors = STM_RESTRICTED_SECTORS
    elif self.flash_type == "M25":
      self.flash_type_byte = 1
      self.addr_sector_map = m25_addr_sector_map
      # Add M25-specific functions.
      self.__dict__['write_status'] = \
          new.instancemethod(_m25_write_status, self, Flash)
      self.n_sectors = M25_N_SECTORS
      self.restricted_sectors = M25_RESTRICTED_SECTORS
    else:
      raise ValueError("flash_type must be \"STM\" or \"M25\"")

  def inc_n_queued_ops(self):
    self.nqo_lock.acquire()
    self._n_queued_ops += 1
    self.nqo_lock.release()

  def dec_n_queued_ops(self):
    self.nqo_lock.acquire()
    self._n_queued_ops -= 1
    self.nqo_lock.release()

  def get_n_queued_ops(self):
    return self._n_queued_ops

  def __del__(self):
    if not self.stopped:
      self.stop()

  def stop(self):
    self.stopped = True
    self.link.rm_callback(SBP_MSG_FLASH_DONE, self._done_callback)
    self.link.rm_callback(SBP_MSG_FLASH_READ, self._read_callback)

  def __str__(self):
    return self.status

  def erase_sector(self, sector, warn=True):
    if warn and sector in self.restricted_sectors:
      text = 'Attempting to erase %s flash restricted sector %d' % \
             (self.flash_type, sector)
      raise Warning(text)
    msg_buf = struct.pack("BB", self.flash_type_byte, sector)
    self.inc_n_queued_ops()
    self.link.send_message(SBP_MSG_FLASH_ERASE, msg_buf)
    while self.get_n_queued_ops() > 0:
      time.sleep(0.001)

  def program(self, address, data):
    msg_buf = struct.pack("B", self.flash_type_byte)
    msg_buf += struct.pack("<I", address)
    msg_buf += struct.pack("B", len(data))
    self.inc_n_queued_ops()
    self.link.send_message(SBP_MSG_FLASH_PROGRAM, msg_buf + data)

  def read(self, address, length):
    msg_buf = struct.pack("B", self.flash_type_byte)
    msg_buf += struct.pack("<I", address)
    msg_buf += struct.pack("B", length)
    self.inc_n_queued_ops()
    self.link.send_message(SBP_MSG_FLASH_READ, msg_buf)

  # Returned for all commands other than read. Returned for read if failed.
  def _done_callback(self, sbp_msg):
    ret = ord(sbp_msg.payload)

    if (ret != 0):
      print "Flash operation returned error (%d)" % ret

    self.dec_n_queued_ops()
    assert self.get_n_queued_ops() >= 0, \
      "Number of queued flash operations is negative"

  # Returned for read if successful.
  def _read_callback(self, sbp_msg):
    # 4 bytes addr, 1 byte length, length bytes data
    address = struct.unpack('<I', data[0:4])[0]
    length = struct.unpack('B', data[4])[0]

    self._read_callback_ihx.puts(address, data[5:])

    self.dec_n_queued_ops()
    assert self.get_n_queued_ops() >= 0, \
      "Number of queued flash operations is negative"

  def write_ihx(self, ihx, stream=None, mod_print=0):
    self.ihx_total_ops = ihx_n_ops(ihx, self.addr_sector_map)
    self.ihx_elapsed_ops = 0
    self.print_count = 0

    start_time = time.time()

    # Erase sectors
    ihx_addrs = ihx_ranges(ihx)
    for sector in sectors_used(ihx_addrs, self.addr_sector_map):
      self.status = self.flash_type + " Flash: Erasing sector %d" % sector
      if stream:
        stream.write('\r' + self.status)
        stream.flush()
      self.erase_sector(sector)
      self.ihx_elapsed_ops += 1
    if stream:
      stream.write('\n')

    # Write data to flash and read back to later validate. STM's lowest address
    # is used by bootloader to check that the application is valid, so program
    # from high to low to ensure this address is programmed last.
    for start, end in reversed(ihx_addrs):
      for addr in reversed(range(start, end, ADDRS_PER_OP)):
        self.status = self.flash_type + " Flash: Programming address" + \
                                        " 0x%08X" % addr
        if stream:
          if mod_print == 0 or mod_print != 0 and self.print_count % mod_print == 0:
            stream.write('\r' + self.status)
            stream.flush()
            self.print_count = 1
          else:
            self.print_count += 1

        binary = ihx.tobinstr(start=addr, size=ADDRS_PER_OP)

        while self.get_n_queued_ops() >= MAX_QUEUED_OPS:
          time.sleep(0.001)
        self.program(addr, binary)
        self.ihx_elapsed_ops += 1

        while self.get_n_queued_ops() >= MAX_QUEUED_OPS:
          time.sleep(0.001)
        self.read(addr, ADDRS_PER_OP)
        self.ihx_elapsed_ops += 1

    # Verify that data written to flash matches data read from flash.
    while self.get_n_queued_ops() > 0:
      time.sleep(0.001)
    for start, end in reversed(ihx_addrs):
      if self._read_callback_ihx.gets(start, end-start+1) != \
          ihx.gets(start, end-start+1):
        for i in range(start, end):
          r = self._read_callback_ihx.gets(i,1)
          p = ihx.gets(i,1)
          if r != p:
            raise Exception('Data read from flash != Data programmed to flash'
              ('Addr: %x, Programmed: %x, Read: %x' % (i,r,p)).upper())

    self.status = self.flash_type + " Flash: Successfully programmed and " + \
                                    "verified, total time = %d seconds" % \
                                    int(time.time()-start_time)
    if stream:
      stream.write('\n\r' + self.status + '\n')


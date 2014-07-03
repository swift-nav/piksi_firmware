#!/usr/bin/env python
# Copyright (C) 2014 Swift Navigation Inc.
# Contact: Gareth McMullin <gareth@swift-nav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

import struct
import sys
import serial_link
import sbp_piksi as ids
import threading

class FileIO(object):
  def __init__(self, link):
    self.link = link

  def read(self, filename):
    chunksize = 255 - 6 - len(filename)
    buf = ''
    while True:
      msg = struct.pack("<IB", len(buf), chunksize) + filename + '\0'
      self.link.send_message(ids.FILEIO_READ, msg)
      data = self.link.wait_message(ids.FILEIO_READ, timeout=1.0)
      if not data:
        raise Exception("Timeout waiting for FILEIO_READ reply")
      if data[:len(msg)] != msg:
        raise Exception("Reply FILEIO_READ doesn't match request")
      chunk = data[len(msg):]
      buf += chunk
      if len(chunk) != chunksize:
        return buf

  def readdir(self, dirname='.'):
    files = []
    while True:
      msg = struct.pack("<I", len(files)) + dirname + '\0'
      self.link.send_message(ids.FILEIO_READ_DIR, msg)
      data = self.link.wait_message(ids.FILEIO_READ_DIR, timeout=1.0)
      if not data:
        raise Exception("Timeout waiting for FILEIO_READ_DIR reply")
      if data[:len(msg)] != msg:
        raise Exception("Reply FILEIO_READ_DIR doesn't match request")
      chunk = data[len(msg):].split('\0')
      files += chunk[:-1]
      if chunk[-1] == '\xff':
        return files

  def remove(self, filename):
    self.link.send_message(ids.FILEIO_REMOVE, filename + '\0')

  def write(self, filename, data, offset=0, trunc=True):
    if trunc and offset == 0:
      self.remove(filename)
    chunksize = 255 - len(filename) - 5
    while data:
      chunk = data[:chunksize]
      data = data[chunksize:]
      header = struct.pack("<I", offset) + filename + '\0'
      self.link.send_message(ids.FILEIO_WRITE, header + chunk)
      reply = self.link.wait_message(ids.FILEIO_WRITE, timeout=1.0)
      if not reply:
        raise Exception("Timeout waiting for FILEIO_WRITE reply")
      if reply != header:
        raise Exception("Reply FILEIO_WRITE doesn't match request")
      offset += len(chunk)

def hd(data):
  ret = ''
  ofs = 0
  while data:
    chunk = data[:16]
    data = data[16:]
    s = "%08X  " % ofs
    s += " ".join("%02X" % ord(c) for c in chunk[:8]) + "  "
    s += " ".join("%02X" % ord(c) for c in chunk[8:])
    s += "".join(" " for i in range(60 - len(s))) + "|"
    for c in chunk:
      s += c if 32 <= ord(c) < 128 else '.'
    s += '|\n'
    ofs += 16
    ret += s
  return ret

if __name__ == "__main__":
  f = FileIO(serial_link.SerialLink())
  try:
    if len(sys.argv) > 1:
      data = f.read(sys.argv[1])
      print hd(data)
    else:
      print f.readdir()
  except KeyboardInterrupt:
    pass


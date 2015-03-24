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
import threading

from sbp.piksi import *

class FileIO(object):
  def __init__(self, link):
    self.link = link

  def read(self, filename):
    """
    Read the contents of a file.

    Parameters
    ----------
    filename : str
        Name of the file to read.

    Returns
    -------
    out : str
        Contents of the file.
    """
    chunksize = 255 - 6 - len(filename)
    buf = ''
    while True:
      msg = struct.pack("<IB", len(buf), chunksize) + filename + '\0'
      self.link.send_message(SBP_MSG_FILEIO_READ, msg)
      data = self.link.wait_message(SBP_MSG_FILEIO_READ, timeout=1.0)
      if not data:
        raise Exception("Timeout waiting for FILEIO_READ reply")
      if data[:len(msg)] != msg:
        raise Exception("Reply FILEIO_READ doesn't match request")
      chunk = data[len(msg):]
      buf += chunk
      if len(chunk) != chunksize:
        return buf

  def readdir(self, dirname='.'):
    """
    List the files in a directory.

    Parameters
    ----------
    dirname : str (optional)
        Name of the directory to list. Defaults to the root directory.

    Returns
    -------
    out : [str]
        List of file names.
    """
    files = []
    while True:
      msg = struct.pack("<I", len(files)) + dirname + '\0'
      self.link.send_message(SBP_MSG_FILEIO_READ_DIR, msg)
      data = self.link.wait_message(SBP_MSG_FILEIO_READ_DIR, timeout=1.0)
      if not data:
        raise Exception("Timeout waiting for FILEIO_READ_DIR reply")
      if data[:len(msg)] != msg:
        raise Exception("Reply FILEIO_READ_DIR doesn't match request")
      chunk = data[len(msg):].split('\0')
      files += chunk[:-1]
      if chunk[-1] == '\xff':
        return files

  def remove(self, filename):
    """
    Delete a file.

    Parameters
    ----------
    filename : str
        Name of the file to delete.
    """
    self.link.send_message(SBP_MSG_FILEIO_REMOVE, filename + '\0')

  def write(self, filename, data, offset=0, trunc=True):
    """
    Write to a file.

    Parameters
    ----------
    filename : str
        Name of the file to write to.
    data : str
        Data to write
    offset : int (optional)
        Offset into the file at which to start writing in bytes.
    trunc : bool (optional)
        Overwite the file, i.e. delete any existing file before writing. If
        this option is not specified and the existing file is longer than the
        current write then the contents of the file beyond the write will
        remain. If offset is non-zero then this flag is ignored.

    Returns
    -------
    out : str
        Contents of the file.
    """
    if trunc and offset == 0:
      self.remove(filename)
    chunksize = 255 - len(filename) - 5
    while data:
      chunk = data[:chunksize]
      data = data[chunksize:]
      header = struct.pack("<I", offset) + filename + '\0'
      self.link.send_message(SBP_MSG_FILEIO_WRITE, header + chunk)
      reply = self.link.wait_message(SBP_MSG_FILEIO_WRITE, timeout=1.0)
      if not reply:
        raise Exception("Timeout waiting for FILEIO_WRITE reply")
      if reply != header:
        raise Exception("Reply FILEIO_WRITE doesn't match request")
      offset += len(chunk)

def hexdump(data):
  """
  Print a hex dump.

  Parameters
  ----------
  data : indexable
      Data to display dump of, can be anything that supports length and index
      operations.
  """
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

def print_dir_listing(files):
  """
  Print a directory listing.

  Parameters
  ----------
  files : [str]
      List of file names in the directory.
  """
  for f in files:
    print f


if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(description='Swift Nav File I/O Utility.')
  parser.add_argument('-r', '--read', nargs=1,
                     help='read a file')
  parser.add_argument('-l', '--list', default=None, nargs=1,
                     help='list a directory')
  parser.add_argument('-d', '--delete', nargs=1,
                     help='delete a file')
  parser.add_argument('-p', '--port',
                     default=[serial_link.DEFAULT_PORT], nargs=1,
                     help='specify the serial port to use.')
  parser.add_argument("-b", "--baud",
                     default=[serial_link.DEFAULT_BAUD], nargs=1,
                     help="specify the baud rate to use.")
  parser.add_argument("-v", "--verbose",
                     help="print extra debugging information.",
                     action="store_true")
  parser.add_argument("-x", "--hex",
                     help="output in hex dump format.",
                     action="store_true")
  parser.add_argument("-f", "--ftdi",
                     help="use pylibftdi instead of pyserial.",
                     action="store_true")
  args = parser.parse_args()
  serial_port = args.port[0]
  baud = args.baud[0]
  link = serial_link.SerialLink(serial_port, baud, use_ftdi=args.ftdi,
                                print_unhandled=args.verbose)

  f = FileIO(link)

  try:
    if args.read:
      data = f.read(args.read[0])
      if args.hex:
        print hexdump(data)
      else:
        print data
    elif args.delete:
      f.remove(args.delete[0])
    elif args.list is not None:
      print_dir_listing(f.readdir(args.list[0]))
    else:
      print "No command given, listing root directory:"
      print_dir_listing(f.readdir())
  except KeyboardInterrupt:
    pass


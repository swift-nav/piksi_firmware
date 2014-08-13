#!/usr/bin/env python
# Copyright (C) 2011-2014 Swift Navigation Inc.
# Contact: Fergus Noble <fergus@swift-nav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

import re
import struct
import threading
import time
import sys

import sbp_piksi as ids


DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 1000000

SBP_PREAMBLE = 0x55

crc16_tab = [
  0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
  0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
  0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
  0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
  0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
  0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
  0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
  0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
  0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
  0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
  0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
  0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
  0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
  0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
  0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
  0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
  0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
  0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
  0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
  0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
  0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
  0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
  0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
  0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
  0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
  0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
  0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
  0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
  0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
  0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
  0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
  0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
]

# CRC16 implementation acording to CCITT standards.
def crc16(s, crc=0):
  for ch in s:
    crc = ((crc<<8)&0xFFFF) ^ crc16_tab[ ((crc>>8)&0xFF) ^ (ord(ch)&0xFF) ]
    crc &= 0xFFFF
  return crc

class ListenerThread (threading.Thread):

  def __init__(self, link, print_unhandled=False):
    super(ListenerThread, self).__init__()
    self.link = link
    self.wants_to_stop = False
    self.print_unhandled = print_unhandled
    self.daemon = True

  def stop(self):
    self.wants_to_stop = True

  def run(self):
    while not self.wants_to_stop:
      try:
        mt, ms, md = self.link.get_message()
        if self.wants_to_stop: #will throw away last message here even if it is valid
          if self.link.ser:
            self.link.ser.close()
          break
        if mt is not None:
          cbs = self.link.get_callback(mt)
          if cbs is None or len(cbs) == 0:
            if self.print_unhandled:
              print "Host Side Unhandled message %02X" % mt
          else:
            for cb in cbs:
              try:
                cb(md, sender=ms)
              except TypeError:
                cb(md)
      except Exception, err:
        import traceback
        print traceback.format_exc()
        return

def list_ports(self=None):
  import serial.tools.list_ports
  return serial.tools.list_ports.comports()

class SerialLink:

  def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, use_ftdi=False, print_unhandled=False):
    self.print_unhandled = print_unhandled
    self.unhandled_bytes = 0
    self.callbacks = {}
    if use_ftdi:
      import pylibftdi
      self.ser = pylibftdi.Device()
      self.ser.baudrate = baud
    else:
      import serial
      try:
        self.ser = serial.Serial(port, baud, timeout=1)
      except serial.SerialException:
        print
        print "Serial device '%s' not found" % port
        print
        print "The following serial devices were detected:"
        print
        for p in list_ports():
          p_name, p_desc, _ = p
          if p_desc == p_name:
            print "\t%s" % p_name
          else:
            print "\t%s (%s)" % (p_name, p_desc)
        sys.exit(1)

    # Delay then flush the buffer to make sure the receive buffer starts empty.
    time.sleep(0.5)
    self.ser.flush()

    self.lt = ListenerThread(self, print_unhandled)
    self.lt.start()

  def __del__(self):
    self.close()

  def close(self):
    try:
      self.lt.stop()
    except AttributeError:
      pass

  def get_message(self):
    while True:
      if self.lt.wants_to_stop:
        return (None, None, None)

      # Sync with magic start bytes
      try:
        magic = self.ser.read(1)
      except OSError:
        print "Error: Piksi not connected"
        sys.exit(1)
      if magic:
        if ord(magic) == SBP_PREAMBLE:
          break
        else:
          self.unhandled_bytes += 1
          if self.print_unhandled:
            print "Host Side Unhandled byte : 0x%02x," % ord(magic), \
                                                            "total", \
                                               self.unhandled_bytes

    hdr = ""
    while len(hdr) < 5:
      hdr = self.ser.read(5 - len(hdr))

    msg_type, sender_id, msg_len = struct.unpack('<HHB', hdr)
    crc = crc16(hdr, 0)

    data = ""
    while len(data) < msg_len:
      data += self.ser.read(msg_len - len(data))
    crc = crc16(data, crc)

    crc_received = ""
    while len(crc_received) < 2:
      crc_received = self.ser.read(2 - len(crc_received))
    crc_received = struct.unpack('<H', crc_received)[0]

    if crc != crc_received:
      print "Host Side CRC mismatch: 0x%04X 0x%04X" % (crc, crc_received)
      return (None, None, None)

    return (msg_type, sender_id, data)

  def send_message(self, msg_type, msg, sender_id=0x42):
    framed_msg = struct.pack('<BHHB', SBP_PREAMBLE, msg_type, sender_id, len(msg))
    framed_msg += msg
    crc = crc16(framed_msg[1:], 0)
    framed_msg += struct.pack('<H', crc)

    self.ser.write(framed_msg)

  def send_char(self, char):
    self.ser.write(char)

  def add_callback(self, msg_type, callback):
    try:
      self.callbacks[msg_type].append(callback)
    except KeyError:
      self.callbacks[msg_type] = [callback]

  def rm_callback(self, msg_type, callback):
    try:
      self.callbacks[msg_type].remove(callback)
    except KeyError:
      print "Can't remove callback for msg 0x%04x: message not registered" \
            % msg_type
    except ValueError:
      print "Can't remove callback for msg 0x%04x: callback not registered" \
            % msg_type

  def get_callback(self, msg_type):
    if msg_type in self.callbacks:
      return self.callbacks[msg_type]
    else:
      return None

def default_print_callback(data):
  sys.stdout.write(data)

if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(description='Swift Nav Serial Link.')
  parser.add_argument('-p', '--port',
                     default=[DEFAULT_PORT], nargs=1,
                     help='specify the serial port to use.')
  parser.add_argument("-b", "--baud",
                     default=[DEFAULT_BAUD], nargs=1,
                     help="specify the baud rate to use.")
  parser.add_argument("-v", "--verbose",
                     help="print extra debugging information.",
                     action="store_true")
  parser.add_argument("-f", "--ftdi",
                     help="use pylibftdi instead of pyserial.",
                     action="store_true")
  args = parser.parse_args()
  serial_port = args.port[0]
  baud = args.baud[0]
  link = SerialLink(serial_port, baud, use_ftdi=args.ftdi,
                    print_unhandled=args.verbose)
  link.add_callback(ids.PRINT, default_print_callback)
  try:
    while True:
      time.sleep(0.1)
  except KeyboardInterrupt:
    pass
  finally:
    link.close()

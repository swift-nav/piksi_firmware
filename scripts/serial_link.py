#!/usr/bin/env python

import re
import struct
import threading
import time
import sys


DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 230400

DEBUG_MAGIC_1 = 0xBE
DEBUG_MAGIC_2 = 0xEF

MSG_PRINT = 0x01

class ListenerThread (threading.Thread):
  wants_to_stop = False

  def __init__(self, link):
    super(ListenerThread, self).__init__()
    self.link = link

  def stop(self):
    self.wants_to_stop = True

  def run(self):
    while not self.wants_to_stop:
      try:
        mt, md = self.link.get_message()
        if self.wants_to_stop: #will throw away last message here even if it is valid
          if self.link.ser:
            self.link.ser.close()
          break
        cb = self.link.get_callback(mt)
        if cb:
          cb(md)
        else:
          print "Unhandled message %02X" % mt
      except Exception, err:
        import traceback
        print traceback.format_exc()

class SerialLink:
  unhandled_bytes = 0
  callbacks = {}

  def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, use_ftdi=False):
    if use_ftdi:
      import pylibftdi
      self.ser = pylibftdi.Device()
      self.ser.baudrate = baud
    else:
      import serial
      self.ser = serial.Serial(port, baud, timeout=1)

    self.lt = ListenerThread(self)
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
        return (None, None)
      # Sync with magic start bytes
      magic = self.ser.read(1)
      if magic:
        if ord(magic) == DEBUG_MAGIC_1:
          magic = self.ser.read(1)
          if ord(magic) == DEBUG_MAGIC_2:
            break
          else:
            self.unhandled_bytes += 1
            print "Unhandled byte : 0x%02x," % ord(magic), "total", self.unhandled_bytes
        else:
          self.unhandled_bytes += 1
          print "Unhandled byte : 0x%02x," % ord(magic), "total", self.unhandled_bytes
    msg_type = ord(self.ser.read(1))
    msg_len = ord(self.ser.read(1))
    data = ""
    while len(data) < msg_len:
      data += self.ser.read(msg_len - len(data))
    return (msg_type, data)

  def send_message(self, msg_type, msg):
    self.ser.write(chr(DEBUG_MAGIC_1))
    self.ser.write(chr(DEBUG_MAGIC_2))
    self.ser.write(chr(msg_type))
    self.ser.write(chr(len(msg)))
    self.ser.write(msg)

  def add_callback(self, msg_type, callback):
    self.callbacks[msg_type] = callback

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
  parser.add_argument("-f", "--ftdi",
                    help="use pylibftdi instead of pyserial.",
                    action="store_true")
  args = parser.parse_args()
  serial_port = args.port[0]
  link = SerialLink(serial_port, use_ftdi=args.ftdi)
  link.add_callback(MSG_PRINT, default_print_callback)
  try:
    while True:
      time.sleep(1)
  except KeyboardInterrupt:
    pass
  finally:
    link.close()

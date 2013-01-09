#!/usr/bin/env python

import serial
import re
import struct
import threading
import time
import sys

DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 921600

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

class SerialLink:
  unhandled_bytes = 0
  callbacks = {}

  def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD):
    self.ser = serial.Serial(port, baud, timeout=1)
    self.lt = ListenerThread(self)
    self.lt.start()

  def __del__(self):
    self.close()

  def close(self):
    self.lt.stop()

  def get_message(self):
    while True:
      if self.lt.wants_to_stop:
        return (None, None)
      # Sync with magic start bytes
      magic = self.ser.read()
      if magic:
        if ord(magic) == DEBUG_MAGIC_1:
          magic = self.ser.read()
          if ord(magic) == DEBUG_MAGIC_2:
            break
          else:
            self.unhandled_bytes += 1
        else:
          self.unhandled_bytes += 1
    msg_type = ord(self.ser.read())
    msg_len = ord(self.ser.read())
    data = ""
    while len(data) < msg_len:
      data += self.ser.read(msg_len - len(data))
    return (msg_type, data)

  def send_message(self, msg_type, msg):
    #print "Sending, id=0x%02X, len=%d" % (msg_type, len(msg))
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
  args = parser.parse_args()
  serial_port = args.port[0]
  link = SerialLink(serial_port)
  link.add_callback(MSG_PRINT, default_print_callback)
  try:
    while True:
      time.sleep(1)
  except KeyboardInterrupt:
    pass
  finally:
    link.close()


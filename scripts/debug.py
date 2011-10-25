#!/usr/bin/env python

import serial
import re
import struct
import threading
import time
import sys

DEBUG_MAGIC_1 = 0xBE
DEBUG_MAGIC_2 = 0xEF

MSG_PRINT = 0x01

parse_re = re.compile("#define MSG_([A-Z0-9i_]+) (0x[0-9][0-9]) // (.*)")
struct_re = re.compile("struct '(.*)'")

def parse_messages(filename):
  messages = {}
  f = open(filename, 'r')
  messages_header = f.read()
  matches = parse_re.findall(messages_header)
  for match in matches:
    msg_type_name = match[0].lower()
    msg_type_num = int(match[1], 16)
    msg_format = match[2]
    messages[msg_type_num] = (msg_type_name, msg_format)
  f.close()
  return messages

def get_message(serial):
  # Sync with magic start bytes
  while True:
    foo = ser.read()
    if foo and ord(foo) == DEBUG_MAGIC_1:
      if ord(ser.read()) == DEBUG_MAGIC_2:
        break
  msg_type = ord(ser.read())
  msg_len = ord(ser.read())
  data = ser.read(msg_len)
  return (msg_type, msg_len, data)

def extract_message(msg_type, msg_len, msg_data, messages):
  msg_type_name, msg_format = messages[msg_type]
  if msg_format == 'string':
    return msg_data
  elif msg_format[:6] == 'struct':
    struct_format = struct_re.match(msg_format).groups(1)[0]
    return struct.unpack(struct_format, msg_data)

def find_message_type_by_name(msg_type_name, messages):
  for this_msg_type, this_msg in messages.iteritems():
    this_msg_type_name = this_msg[0]
    if this_msg_type_name == msg_type_name:
      return this_msg_type
  return None

def construct_message(msg_type_name, args, messages):
  msg_type = find_message_type_by_name(msg_type_name, messages)
  msg_format = messages[msg_type][1]
  if msg_format == 'string':
    return args
  elif msg_format[:6] == 'struct':
    struct_format = struct_re.match(msg_format).groups(1)[0]
    pack_args = [struct_format] + args
    return struct.pack(*pack_args)
  elif msg_format == 'none':
    return []

def send_message(serial, msg_type, msg):
  print "Sending, id=0x%02X, len=%d" % (msg_type, len(msg))
  serial.write(chr(DEBUG_MAGIC_1))
  serial.write(chr(DEBUG_MAGIC_2))
  serial.write(chr(msg_type))
  serial.write(chr(len(msg)))
  serial.write(msg)

ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
messages =  parse_messages('../src/debug_messages.h')

class ListenerThread (threading.Thread):
  wants_to_stop = False

  def stop(self):
    self.wants_to_stop = True

  def run(self):
    while(not self.wants_to_stop):
      mt, ml, md = get_message(ser)
      if mt == MSG_PRINT:
        sys.stdout.write("\x1b[34m" + md + "\x1b[0m")
      else:
        print "%s: %s" % (messages[mt][0], str(extract_message(mt, ml, md, messages)))

def send_msg(ser, msg_type_name, args):
  msg_type = find_message_type_by_name(msg_type_name, messages)
  send_message(ser, msg_type, construct_message(msg_type_name, args, messages))


lt = ListenerThread()

try:
  lt.start()
  #send_msg(ser, 'flash_erase_all', [])
  #send_msg(ser, 'flash_write', [0x00, 22] + 250*[0x69])
  while(1):
    #send_msg(ser, 'flash_read', [0x00, 22])
    time.sleep(1)
except KeyboardInterrupt:
  pass
finally:
  lt.stop()
  #lt.join()
  ser.close()


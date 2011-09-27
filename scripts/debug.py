#!/usr/bin/env python

import serial
import re
import struct

DEBUG_MAGIC_1 = 0xBE
DEBUG_MAGIC_2 = 0xEF

MSG_PRINT = 0x01

parse_re = re.compile("#define MSG_([A-Z0-9]+) (0x[0-9][0-9]) // (.*)")
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
    if ord(ser.read()) == DEBUG_MAGIC_1:
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

ser = serial.Serial('/dev/tty.usbserial-FJ000047B', 9600, timeout=2)
messages =  parse_messages('../src/debug_messages.h')

try:
  while(1):
    mt, ml, md = get_message(ser)
    if mt == MSG_PRINT:
      print md,
    else:
      print "%s: %s" % (messages[mt][0], str(extract_message(mt, ml, md, messages)))
except KeyboardInterrupt:
  pass
finally:
  ser.close()


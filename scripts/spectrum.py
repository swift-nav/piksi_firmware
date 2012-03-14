#!/usr/bin/env python

import serial
import re
import struct
import threading
import time
import sys
import string
import pylab

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

ser = serial.Serial('/dev/ttyUSB1', 921600, timeout=1)
messages =  parse_messages('../src/debug_messages.h')

class ListenerThread (threading.Thread):
  wants_to_stop = False
  prep_plot_vals = False

  def stop(self):
    self.wants_to_stop = True

  def run(self):
    pylab.figure(1)
    pylab.ion()
    pylab.xlabel("Hz")
    pylab.ylabel("Magnitude")
    pylab.title("CW around DC")
    num_avgs = 40
    #Find number of freq points
    freq_pts = 0
    mt, ml, md = get_message(ser)
    while not(re.search("#PLOT_DATA_START",md)):
      freq_pts = 0
      mt, ml, md = get_message(ser)
    while not((re.search("#PLOT_DATA_END",md))):
      if bool(re.match("[-+]\d+.\d+ \d+$",md)):
        freq_pts += 1
      mt, ml, md = get_message(ser)
    #Initialize arrays
    freqs = pylab.zeros(freq_pts)
    mags = pylab.zeros((num_avgs,freq_pts))
    avg_mags = pylab.zeros(freq_pts)
    avg_count = 0
    freq_count = 0
    plot_y_max = 0
    plot_y_min = pylab.inf
    freq_pts = 0
    while(not self.wants_to_stop):
      mt, ml, md = get_message(ser)
      if mt == MSG_PRINT:
#          sys.stdout.write("\x1b[34m" + md + "\x1b[0m")
        sys.stdout.write(md)
        if (re.search("#PLOT_DATA_START",md)):
          self.prep_plot_vals = True
          freq_count = 0
#        elif self.prep_plot_vals and bool(re.match("[-+]\d+.\d+ \d+ # \d+$",md)):
        elif self.prep_plot_vals and bool(re.match("[-+]\d+.\d+ \d+$",md)):
          freqs[freq_count] = float(string.split(md," ")[0])
          avg_mags[freq_count] = avg_mags[freq_count]-mags[avg_count][freq_count]
          mags[avg_count][freq_count] = float(string.split(md," ")[1])
          avg_mags[freq_count] = avg_mags[freq_count]+mags[avg_count][freq_count]
          freq_count += 1
        elif self.prep_plot_vals and (re.search("#PLOT_DATA_END",md)):
          self.prep_plot_vals = False
          max_avgs = 0
          max_freq = 0
          for i in range(0,len(avg_mags)):
            if (avg_mags[i] > plot_y_max):
              plot_y_max = avg_mags[i]
            if (avg_mags[i] > max_avgs):
              max_avgs = avg_mags[i]
              max_freq = freqs[i]
          plot_y_min = 2*pylab.mean(avg_mags) - plot_y_max
          freqs_mhz = freqs/1e6
          pylab.clf()
#          pylab.plot(freqs_mhz,avg_mags)
          pylab.semilogy(freqs_mhz,avg_mags)
          pylab.axis([min(freqs_mhz),max(freqs_mhz),plot_y_min,plot_y_max])
          pylab.xlabel("MHz")
          pylab.ylabel("Magnitude")
          pylab.title("Spectrum")
          pylab.draw()
          print str(avg_count)
          print "  max freq =", str((1.57542e9 + max_freq)/1e9), "GHz"
          print "  max freq offset from carrier=", str(max_freq)
          if (avg_count == (num_avgs-1)):
            avg_count = 0
          else:
            avg_count += 1
#      else:
#        print "%s: %s" % (messages[mt][0], str(extract_message(mt, ml, md, messages)))

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


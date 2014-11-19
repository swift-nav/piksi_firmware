#!/usr/bin/env python

import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

import serial_link
import settings_view
import argparse
import time
import pprint

parser = argparse.ArgumentParser(description='Print Piksi device details.')
parser.add_argument('-p', '--port',
	     default=[serial_link.DEFAULT_PORT], nargs=1,
	     help='specify the serial port to use.')
parser.add_argument("-b", "--baud",
	     default=[serial_link.DEFAULT_BAUD], nargs=1,
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
link = serial_link.SerialLink(serial_port, baud, use_ftdi=args.ftdi,
	    print_unhandled=args.verbose)

settings_read = False
def callback():
  global settings_read
  settings_read = True

sv = settings_view.SettingsView(link, read_finished_functions=[callback], gui_mode=False)

while not settings_read:
  time.sleep(1)
  print settings_read

print "===================================="
print "Piksi Device", serial_port
print "===================================="
print
print "System Info"
print "-----------"
print

for k_, v_ in sv.settings['system_info'].iteritems():
  print "%-20s %s" % (k_, v_)

print
print "Settings"
print "--------"
print

for k, v in sv.settings.iteritems():
  if k != 'system_info':
    print "%s:" % k
    for k_, v_ in v.iteritems():
      print "    %-20s %s" % (k_, v_)
    print

link.close()


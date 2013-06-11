#!/usr/bin/env python

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'scripts'))
import serial_link

link = serial_link.SerialLink()

link.add_callback(serial_link.MSG_PRINT, serial_link.default_print_callback)

data = ''.join(map(chr, range(0, 22)))

try:
  while True:
    time.sleep(0.02)
    link.send_message(0x22, data)
except KeyboardInterrupt:
  pass
finally:
  link.close()


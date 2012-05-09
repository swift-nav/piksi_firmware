#!/usr/bin/env python

import sys, os, time
import binascii
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'scripts'))

import serial_link

ok_count = 0
ok_byte_count = 0
def foo_cb(data):
  global ok_count
  global ok_byte_count

  for n, d in enumerate(data):
    if ord(d) != ((n) % 256):
      print n
      print binascii.hexlify(data)[:2*n]
      print binascii.hexlify(data)[2*n:]
      print "EIT!"
      return
  ok_count += 1
  ok_byte_count += len(data)

link = serial_link.SerialLink()

link.add_callback(serial_link.MSG_PRINT, serial_link.default_print_callback)

link.add_callback(0x22, foo_cb)

try:
  old = 0
  while True:
    time.sleep(1)
    kbs = (ok_byte_count - old) / 1024.0
    if ok_byte_count != old:
      print "%d Messages (%d kB) %.2f kB/s" % (ok_count, ok_byte_count/1024, kbs)
    old = ok_byte_count
except KeyboardInterrupt:
  pass
finally:
  link.close()


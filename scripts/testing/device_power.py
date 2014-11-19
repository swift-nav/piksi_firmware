#!/usr/bin/env python

# Switch the power on/off to a USB device
# connected to a Digi HUBPORT/14
#
# Usage:
# ./device_power.py /dev/ttyUSB0 0
# ./device_power.py /dev/ttyUSB0 1

import pyudev
import sys
import subprocess

device_file = sys.argv[1]
power = int(sys.argv[2]) == 1

c = pyudev.Context()
d = pyudev.Device.from_device_file(c, device_file)
d_child = d

while d is not None:
  if d.subsystem == "usb" and d.driver == "usb":
    # Look for the first parent that matches the Digi HUBPORT/14
    if d.attributes['idVendor'] == "1608" and \
       d.attributes['idProduct'] == "0186":
      break
  d_child = d
  d = d.parent

bus = int(d.attributes['busnum'])
dev = int( d.attributes['devnum'])
port = int(d_child.attributes['devpath'].split('.')[-1])

ret = subprocess.call("./hub_ctrl -b %d -d %d -P %d -p %d" %
                      (bus, dev, port, 1 if power else 0), shell=True)

sys.exit(ret)


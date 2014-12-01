#!/usr/bin/env python

# Switch the power on/off to a USB device
# connected to a Digi HUBPORT/14
#
# Usage:
# ./device_power.py /dev/ttyUSB0

import pyudev
import sys
import subprocess
import time

device_files = sys.argv[1:]

c = pyudev.Context()

def get_device_location(device_file):
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

  return (bus, dev, port)

# Calculate ports to power cycle
ports = map(get_device_location, device_files)

# Turn off all devices
for bus, dev, port in ports:
  ret = subprocess.call("./hub_ctrl -b %d -d %d -P %d -p 0" %
                        (bus, dev, port), shell=True)
  if ret != 0:
    sys.exit(ret)

time.sleep(2)

# Turn on all devices
for bus, dev, port in ports:
  ret = subprocess.call("./hub_ctrl -b %d -d %d -P %d -p 1" %
                        (bus, dev, port), shell=True)
  if ret != 0:
    sys.exit(ret)


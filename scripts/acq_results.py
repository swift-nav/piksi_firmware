#!/usr/bin/env python
# Copyright (C) 2011-2014 Swift Navigation Inc.
# Contact: Colin Beighley <colin@swift-nav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.


import serial_link
import sbp_piksi as ids
import argparse
import sys
import time
import struct

N_RECORD = 0 # Number of results to keep in memory, 0 = no limit.
N_PRINT = 32

class AcqResults():

  def __init__(self, link):
    self.acqs = []
    self.link = link
    self.link.add_callback(ids.ACQ_RESULT, self._receive_acq_result)
    self.max_corr = 0

  def __str__(self):
    tmp = "Last %d acquisitions:\n" % len(self.acqs[-N_PRINT:])
    for a in self.acqs[-N_PRINT:]:
      tmp += "SV %2d, SNR: %3.2f\n" % (a['SV'], a['SNR'])
    return tmp

  def mean_corr(self):
    if len(self.acqs) == 0:
      return 0
    else:
      return float(sum([a['MC'] for a in self.acqs]))/len(self.acqs)

  def max_snr(self):
    try:
      return max([a['SNR'] for a in self.acqs])
    except ValueError:
      return 0

  def _receive_acq_result(self, data):
    while N_RECORD > 0 and len(self.acqs) >= N_RECORD:
      self.acqs.pop(0)

    self.acqs.append({})
    a = self.acqs[-1]

    a['SNR'] = struct.unpack('f', data[0:4])[0] # SNR of best point.
    a['CP'] = struct.unpack('f', data[4:8])[0]  # Code phase of best point.
    a['CF'] = struct.unpack('f', data[8:12])[0] # Carr freq of best point.
    a['SV'] = struct.unpack('B', data[12])[0]   # SV of acq.

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Acquisition Monitor')
  parser.add_argument("-f", "--ftdi",
                      help="use pylibftdi instead of pyserial.",
                      action="store_true")
  parser.add_argument('-p', '--port',
                      default=[serial_link.DEFAULT_PORT], nargs=1,
                      help='specify the serial port to use.')
  args = parser.parse_args()
  serial_port = args.port[0]

  print "Waiting for device to be plugged in ...",
  sys.stdout.flush()
  found_device = False
  while not found_device:
    try:
      link = serial_link.SerialLink(serial_port, use_ftdi=args.ftdi)
      found_device = True
    except KeyboardInterrupt:
      # Clean up and exit.
      link.close()
      sys.exit()
    except:
      # Couldn't find device.
      time.sleep(0.01)
  print "link with device successfully created."
  link.add_callback(ids.PRINT, serial_link.default_print_callback)

  acq_results = AcqResults(link)

  # Wait for ctrl+C before exiting
  try:
    while True:
      print acq_results
      time.sleep(0.1)
  except KeyboardInterrupt:
    pass

  # Clean up and exit
  link.close()
  sys.exit()

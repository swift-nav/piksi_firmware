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

N_RECORD = 0 # Number of results to keep in memory, 0 = no limit
N_PRINT = 32

class AcqResults():

  def __init__(self, link):
    self.acqs = []
    self.link = link
    self.link.add_callback(MSG_ACQ_RESULT, self._receive_acq_result)
    self.max_corr = 0

  def __str__(self):
    tmp = ""
    tmp += "Last %d acquisitions:\n" % len(self.acqs[-N_PRINT:])
    for a in self.acqs[-N_PRINT:]:
      tmp += "SV %2d, SNR: %3.2f\n" % (a['SV'], a['SNR'])
    tmp += "Max Correlation :  %d\n" % self.max_corr
    tmp += "Mean Correlation : %d\n" % self.mean_corr()
    tmp += "Acq's Received :   %d\n" % len(self.acqs)
    return tmp

  def mean_corr(self):
    if len(self.acqs) == 0:
      return 0
    else:
      return float(sum([a['MC'] for a in self.acqs]))/len(self.acqs)

  def max_snr(self):
    return max([a['SNR'] for a in self.acqs] + [0]) # + [0] otherwise error

  def _receive_acq_result(self, data):
    while N_RECORD > 0 and len(self.acqs) >= N_RECORD:
      self.acqs.pop(0)

    self.acqs.append({})
    a = self.acqs[-1]

    a['SV'] = struct.unpack('B', data[0])[0]        # SV of acq
    a['SNR'] = struct.unpack('f', data[1:5])[0]     # SNR of best point
    a['CP'] = struct.unpack('f', data[5:9])[0]      # Code phase of best point
    a['CF'] = struct.unpack('f', data[9:13])[0]     # Carr freq of best point
    a['BC_I'] = struct.unpack('<i', data[13:17])[0] # Best point I correlation
    a['BC_Q'] = struct.unpack('<i', data[17:21])[0] # Best point Q correlation
    a['MC'] = struct.unpack('<I', data[21:25])[0]   # Mean correlation of acq

    if abs(a['BC_I']) > self.max_corr:
      self.max_corr = abs(a['BC_I'])
    if abs(a['BC_Q']) > self.max_corr:
      self.max_corr = abs(a['BC_Q'])

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
      # Clean up and exit
      link.close()
      sys.exit()
    except:
      # Couldn't find device
      time.sleep(0.01)
  print "link with device successfully created."
  link.add_callback(ids.PRINT, serial_link.default_print_callback)
  #link.add_callback(ids.PRINT, lambda x: None)
  #link.add_callback(ids.BOOTLOADER_HANDSHAKE, lambda x: None)

  acq_results = AcqResults(link)

  # Wait for ctrl+C before exiting
  try:
    while(1):
      print acq_results
      time.sleep(1)
  except KeyboardInterrupt:
    pass

  # Clean up and exit
  link.close()
  sys.exit()

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
import argparse
import sys
import time
import struct

from numpy           import mean
from sbp.acquisition import SBP_MSG_ACQ_RESULT
from sbp.piksi       import SBP_MSG_PRINT

N_RECORD = 0 # Number of results to keep in memory, 0 = no limit.
N_PRINT = 32

SNR_THRESHOLD = 25

class AcqResults():

  def __init__(self, link):
    self.acqs = []
    self.link = link
    self.link.add_callback(SBP_MSG_ACQ_RESULT, self._receive_acq_result)
    self.max_corr = 0

  def __str__(self):
    tmp = "Last %d acquisitions:\n" % len(self.acqs[-N_PRINT:])
    for a in self.acqs[-N_PRINT:]:
      tmp += "PRN %2d, SNR: %3.2f\n" % (a['PRN'], a['SNR'])
    tmp += "Max SNR         : %3.2f\n" % (self.max_snr())
    tmp += "Mean of max SNRs: %3.2f\n" % (self.mean_max_snrs(SNR_THRESHOLD))
    return tmp

  # Return the maximum SNR received.
  def max_snr(self):
    try:
      return max([a['SNR'] for a in self.acqs])
    except ValueError, KeyError:
      return 0

  # Return the mean of the max SNR (above snr_threshold) of each PRN.
  def mean_max_snrs(self, snr_threshold):
    snrs = []
    # Get the max SNR for each PRN.
    for prn in set([a['PRN'] for a in self.acqs]):
      acqs_prn = filter(lambda x: x['PRN'] == prn, self.acqs)
      acqs_prn_max_snr = max([a['SNR'] for a in acqs_prn])
      if acqs_prn_max_snr >= snr_threshold:
        snrs += [max([a['SNR'] for a in acqs_prn])]
    if snrs:
      return mean(snrs)
    else:
      return 0

  def _receive_acq_result(self, sbp_msg):
    while N_RECORD > 0 and len(self.acqs) >= N_RECORD:
      self.acqs.pop(0)

    self.acqs.append({})
    a = self.acqs[-1]

    a['SNR'] = struct.unpack('f', sbp_msg.payload[0:4])[0]  # SNR of best point.
    a['CP'] = struct.unpack('f', sbp_msg.payload[4:8])[0]   # Code phase of best point.
    a['CF'] = struct.unpack('f', sbp_msg.payload[8:12])[0]  # Carr freq of best point.
    a['PRN'] = struct.unpack('B', sbp_msg.payload[12])[0]   # PRN of acq.

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
  link.add_callback(SBP_MSG_PRINT, serial_link.default_print_callback)

  acq_results = AcqResults(link)

  # Wait for ctrl+C before exiting.
  try:
    while True:
      print acq_results
      time.sleep(0.1)
  except KeyboardInterrupt:
    pass

  # Clean up and exit.
  link.close()
  sys.exit()

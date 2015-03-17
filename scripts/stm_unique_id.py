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

import time
import struct
import sys
import serial_link

from sbp.piksi import SBP_MSG_STM_UNIQUE_ID

class STMUniqueID:

  def __init__(self,link):
    self.unique_id_returned = False
    self.unique_id = None
    self.link = link
    link.add_callback(SBP_MSG_STM_UNIQUE_ID, self.receive_stm_unique_id_callback)

  def receive_stm_unique_id_callback(self,data):
    self.unique_id_returned = True
    self.unique_id = struct.unpack('<12B',data.payload)

  def get_id(self):
    self.unique_id_returned = False
    self.unique_id = None
    self.link.send_message(SBP_MSG_STM_UNIQUE_ID, struct.pack("<I",0))
    while not self.unique_id_returned:
      time.sleep(0.1)
    return self.unique_id

if __name__ == "__main__":
  link = serial_link.SerialLink()
  unique_id = STMUniqueID(link).get_id()
  print "STM Unique ID =", "0x" + ''.join(["%02x" % (b) for b in unique_id])
  link.close()

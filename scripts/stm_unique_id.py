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

from sbp.piksi import SBP_MSG_STM_UNIQUE_ID
from sbp.client.main import *

class STMUniqueID:

  def __init__(self,link):
    self.unique_id_returned = False
    self.unique_id = None
    self.link = link
    link.add_callback(self.receive_stm_unique_id_callback, SBP_MSG_STM_UNIQUE_ID)

  def receive_stm_unique_id_callback(self,sbp_msg):
    self.unique_id_returned = True
    self.unique_id = struct.unpack('<12B',sbp_msg.payload)

  def get_id(self):
    self.unique_id_returned = False
    self.unique_id = None
    self.link.send(SBP_MSG_STM_UNIQUE_ID, struct.pack("<I",0))
    while not self.unique_id_returned:
      time.sleep(0.1)
    return self.unique_id

if __name__ == "__main__":
  with get_driver(False, SERIAL_PORT, SERIAL_BAUD) as driver:
    with Handler(driver.read, driver.write) as link:
      unique_id = STMUniqueID(link).get_id()
      print "STM Unique ID =", "0x" + ''.join(["%02x" % (b) for b in unique_id])

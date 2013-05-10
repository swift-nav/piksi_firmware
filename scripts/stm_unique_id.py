#!/usr/bin/env python

import time
import struct
import sys
import serial_link

MSG_STM_UNIQUE_ID = 0xE5

class STMUniqueID:
  unique_id_returned = False
  unique_id = None

  def __init__(self,link):
    self.link = link
    link.add_callback(MSG_STM_UNIQUE_ID, self.receive_stm_unique_id_callback)

  def receive_stm_unique_id_callback(self,data):
    self.unique_id_returned = True
    self.unique_id = struct.unpack('<12B',data)

  def get_id(self):
    self.unique_id_returned = False
    self.unique_id = None
    self.link.send_message(MSG_STM_UNIQUE_ID, struct.pack("<I",0))
    while not self.unique_id_returned:
      time.sleep(0.1)
    return self.unique_id

if __name__ == "__main__":
  link = serial_link.SerialLink()
  unique_id = STMUniqueID(link).get_id()
  print "STM Unique ID =", "0x" + ''.join(["%02x" % (b) for b in unique_id])
  link.close()

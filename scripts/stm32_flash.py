#!/usr/bin/env python

import serial_link
import struct
import time
import sys

MSG_STM_FLASH_PROGRAM  = 0xE0
MSG_STM_FLASH_READ     = 0xE1 #both C and Python ID
MSG_STM_FLASH_ERASE    = 0xE2
MSG_STM_FLASH_COMPLETE = 0xE0

MSG_JUMP_TO_APP = 0xA0

APP_ADDRESS = 0x08010000

#Currently just serves to block until callbacks
#for each command are received
class stm32_flash():
  _waiting_for_callback = False
  _rd_cb_data = 0

  def __init__(self,link):
    self.link = link
    self.link.add_callback(MSG_STM_FLASH_COMPLETE, self._done_callback)
    self.link.add_callback(MSG_STM_FLASH_READ, self._read_callback)

  def erase_sector(self,sector):
    msg_buf = struct.pack("B",sector)
    self._waiting_for_callback = True
    link.send_message(MSG_STM_FLASH_ERASE,msg_buf)
    while self._waiting_for_callback == True:
      pass

  def program_byte(self,address,byte):
    msg_buf = struct.pack("<IB",address,byte)
    self._waiting_for_callback = True
    link.send_message(MSG_STM_FLASH_PROGRAM,msg_buf)
    while self._waiting_for_callback == True:
      pass

  def read_byte(self,address):
    msg_buf = struct.pack("<I",address)
    self._waiting_for_callback = True
    link.send_message(MSG_STM_FLASH_READ,msg_buf)
    while self._waiting_for_callback == True:
      pass
    return self._rd_cb_data

  def _done_callback(self,data):
    self._waiting_for_callback = False

  def _read_callback(self,data):
    self._rd_cb_data = struct.unpack('B',data[0])[0]
    self._waiting_for_callback = False

if __name__ == "__main__":
  #Create serial link to Piksi
  link = serial_link.SerialLink()
  link.add_callback(serial_link.MSG_PRINT, serial_link.default_print_callback)
  #Create stm32_flash object and pass Piksi serial link to it
  flash = stm32_flash(link)

  #Get program binary
#  bin = open(argv[1], "rb").read()
  bin = open("../tests/leds/leds_test.bin", "rb").read()

  #Erase sector of Piksi's flash where binary is to go
  print "Erasing flash ...",
  flash.erase_sector(4)
  print "done"

  #Write program binary to Piksi's flash
  #Read back each address and validate as we go
  addr = APP_ADDRESS
  while bin:
    print ("Programming flash at 0x%08X\r" % addr),
    flash.program_byte(addr,ord(bin[0]))
    byte_read_back = flash.read_byte(addr)
    if byte_read_back != ord(bin[0]):
      raise Exception('Byte read from flash does not match byte written to flash')
    bin = bin[1:]
    addr += 1
  print "\nDone programming flash"

  #Tell STM to jump to application, as programming is finished
  link.send_message(MSG_JUMP_TO_APP,'\x00')

  #Clean up and exit
  link.close()
  sys.exit()

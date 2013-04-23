#!/usr/bin/env python

import serial_link
import struct
import time
import sys
import serial

MSG_STM_FLASH_PROGRAM_BYTE = 0xE0
MSG_STM_FLASH_PROGRAM      = 0xE4
MSG_STM_FLASH_READ         = 0xE1 #both C and Python ID
MSG_STM_FLASH_ERASE_SECTOR = 0xE2
MSG_STM_FLASH_COMPLETE     = 0xE0
MSG_BOOTLOADER_HANDSHAKE   = 0xE3

MSG_JUMP_TO_APP = 0xA0

APP_ADDRESS = 0x08010000
ADDRS_PER_OP = 128

#Currently just serves to block until callbacks
#for each command are received
class stm32_flash():
  bootloader_ready = False
  _waiting_for_callback = False
  _read_callback_data = []

  def __init__(self,link):
    self.link = link
    self.link.add_callback(MSG_STM_FLASH_COMPLETE, self._done_callback)
    self.link.add_callback(MSG_STM_FLASH_READ, self._read_callback)
    self.link.add_callback(MSG_BOOTLOADER_HANDSHAKE, self._bootloader_ready_callback)

  def erase_sector(self,sector):
    msg_buf = struct.pack("B",sector)
    self._waiting_for_callback = True
    link.send_message(MSG_STM_FLASH_ERASE_SECTOR,msg_buf)
    while self._waiting_for_callback == True:
      pass

  def program_byte(self,address,byte):
    msg_buf = struct.pack("<IB",address,byte)
    self._waiting_for_callback = True
    link.send_message(MSG_STM_FLASH_PROGRAM_BYTE,msg_buf)
    while self._waiting_for_callback == True:
      pass

  def program(self,address,data):
    msg_buf = struct.pack("<IB",address,len(data))
    self._waiting_for_callback = True
    link.send_message(MSG_STM_FLASH_PROGRAM,msg_buf+data)
    while self._waiting_for_callback == True:
      pass

  def read(self,address,length):
    msg_buf = struct.pack("<IB",address,length)
    self._waiting_for_callback = True
    link.send_message(MSG_STM_FLASH_READ,msg_buf)
    while self._waiting_for_callback == True:
      pass
    return self._read_callback_data

  def _done_callback(self,data):
    self._waiting_for_callback = False

  def _read_callback(self,data):
    #4 bytes addr, 1 byte length, length bytes data
    addr = struct.unpack('<I',data[0:4])[0];
    length = struct.unpack('B',data[4])[0];
    self._read_callback_data = list(struct.unpack(str(length)+'B',data[5:]))
    self._waiting_for_callback = False

  def _bootloader_ready_callback(self,data):
    self.bootloader_ready = True

if __name__ == "__main__":
  #Wait for device to be plugged in
  print "Waiting for device to be plugged in ...",
  sys.stdout.flush()
  while not locals().has_key('link'): #'link' doesn't exist, it hasn't been created successfully yet, probably because device is not plugged in
    try:
      link = serial_link.SerialLink()
    except serial.serialutil.SerialException: #couldn't find device
      time.sleep(0.1)
  link.add_callback(serial_link.MSG_PRINT, serial_link.default_print_callback)
  print "link with device successfully created."

  #Create stm32_flash object and pass Piksi serial link to it
  flash = stm32_flash(link)

  #Wait until device informs us that it is ready to receive program
  print "Waiting for device to tell us it is ready to bootload ...",
  sys.stdout.flush()
  while not flash.bootloader_ready:
    time.sleep(0.1)
  print "received handshake signal."

  #Send message to device to let it know we want to change the application
  link.send_message(MSG_BOOTLOADER_HANDSHAKE,'\x00')

  #Get program binary
#  binary = open(argv[1], "rb").read()
  binary = open("../tests/leds/leds_test.bin", "rb").read()
#  binary = open("../tests/acq/acq_test.bin", "rb").read()

  #Erase sector of Piksi's flash where binary is to go
  print "Erasing flash ...",
  flash.erase_sector(4)
  print "done"

  #Write program binary to Piksi's flash
  #Read back each address and validate as we go
  addr = APP_ADDRESS
  while binary:
    print ("Programming flash at 0x%08X\r" % addr),
    n_addrs = min([ADDRS_PER_OP,len(binary)])
    flash.program(addr,binary[0:n_addrs])
    flash_readback = flash.read(addr,n_addrs)
    if flash_readback != [ord(i) for i in binary[0:n_addrs]]:
      raise Exception('data read from flash != data written to flash')
    binary = binary[n_addrs:]
    addr += n_addrs
  print "\nDone programming flash, telling device to jump to application"

  #Tell STM to jump to application, as programming is finished
  link.send_message(MSG_JUMP_TO_APP,'\x00')

#  while True:
#    time.sleep(0.1)
#
  #Clean up and exit
  link.close()
  sys.exit()

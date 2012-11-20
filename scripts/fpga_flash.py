#!/usr/bin/env python

import flash
from intelhex import IntelHex
import argparse
import serial_link
import sys
import time

#Command line arguments
parser = argparse.ArgumentParser(description='FPGA Flash Utility')
parser.add_argument('-p', '--port',
                    default=['/dev/ttyUSB0'], nargs=1,
                    help='specify the serial port to use')
parser.add_argument('-c', '--configuration_file',
                    default=['swift-nap_mcs.mcs'], nargs=1,
                    help='hex file to program the flash with')
args = parser.parse_args()
serial_port = args.port[0]
flash_file = args.configuration_file[0]

#Check that ending address of hex file is greater than 16 bytes from
#the end of the flash - we use that space for the FPGA DNA hash
print "Checking to make sure hex file's maximum address is not in last sector"
ihx = IntelHex(flash_file)
assert ihx.maxaddr() < (flash.FLASHSIZE-flash.SECTORSIZE), "Highest address in hexfile in in last sector"

#Create SerialLink and Flash objects to write to FPGA's flash
print "Creating serial link and adding print callback..."
link = serial_link.SerialLink(port=serial_port)
link.add_callback(serial_link.MSG_PRINT, serial_link.default_print_callback)
print "Creating flash object..."
piksi_flash = flash.Flash(link)
piksi_flash.start()

#Write configuration file to FPGA's flash
print "Writing hex file to FPGA's flash..."
piksi_flash.write_ihx(flash_file)

#Wait for flash operations to finish
t1 = time.time()
try:
  while piksi_flash.flash_operations_left() > 0:
    time.sleep(0.1)
except KeyboardInterrupt:
  pass
finally:
  print "Finished writing hex file to device's flash, took %.2f seconds" % (time.time() - t1)
  print "The numbers of commands that were queued =", piksi_flash._num_qd_cmnds
  piksi_flash.stop()
  link.close()
  sys.exit()

print "Finished writing hex file to device's flash, took %.2f seconds" % (time.time() - t1)

#Finished, close the links
try:
  while True:
    time.sleep(0.1)
except KeyboardInterrupt:
  pass
finally:
  print "The numbers of commands that were queued =", piksi_flash._num_qd_cmnds
  piksi_flash.stop()
  link.close()
  sys.exit()

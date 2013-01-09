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

#Check that ending address of hex file is not in the last sector of the flash
ihx = IntelHex(flash_file)
print "Checking to make sure hex file's maximum address is not in last sector"
print "We don't want to erase the device's authentication hash"
print "  First address of flash's last sector =", hex(flash.FLASHSIZE-flash.SECTORSIZE)
print "  Maximum address of hex file          =", hex(ihx.maxaddr())
assert ihx.maxaddr() < (flash.FLASHSIZE-flash.SECTORSIZE), "Maximum address in hex file is in last sector"

#Create SerialLink and Flash objects to write to FPGA's flash
print "Creating serial link and adding print callback..."
link = serial_link.SerialLink(port=serial_port)
link.add_callback(serial_link.MSG_PRINT, serial_link.default_print_callback)
print "Creating flash object..."
piksi_flash = flash.Flash(link)
piksi_flash.start()

#Write configuration file to FPGA's flash
piksi_flash.write_ihx(flash_file)

#Read back the configuration from the flash in order to validate
#print "piksi_flash._read_callbacks_received =", piksi_flash._read_callbacks_received
#print "Reading configuration from flash"
#piksi_flash.read(0,ihx.maxaddr())
#while piksi_flash.flash_operations_left() != 0:
#  try:
#    time.sleep(0.5)
#    if link.unhandled_bytes > 0:
#      print "unhandled bytes =", link.unhandled_bytes
#  except KeyboardInterrupt:
#    break
#print "piksi_flash._read_callbacks_received =", piksi_flash._read_callbacks_received
#print "unhandled bytes received by serial link =", link.unhandled_bytes
#print "piksi_flash.rd_cb_addrs[-1] =", hex(piksi_flash.rd_cb_addrs[-1])
#print "piksi_flash.rd_cb_addrs[-1] + piksi_flash.rd_cb_lens[-1] =", hex(piksi_flash.rd_cb_addrs[-1]+piksi_flash.rd_cb_lens[-1])

#Make sure the starting address and length of each callback we received are continuous
#piksi_flash.read_cb_sanity_check()
#print "Sanity check passed"

#Compare the data read out of the flash with the data in the hex file
#if piksi_flash.rd_cb_data != list(ihx.tobinarray(start=ihx.minaddr(), size=ihx.maxaddr()-ihx.minaddr())):
#  raise Exception('Data read from flash does not match configuration file')
#print "Data read from flash matches configuration file"

#Clean up before exiting
piksi_flash.stop()
link.close()
sys.exit()

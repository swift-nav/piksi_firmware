import struct
import time
from intelhex import IntelHex
from threading import Thread, Event
import sys
from math import ceil

PAGESIZE = 256
SECTORSIZE = 64*1024
FLASHSIZE = 1024*1024

#Maximum number of addresses returned by STM in a single read callback
#Defined in m25_flash.c in the read callback
ADDR_PER_RD_CB = 16

#Maximum number of addresses written in a single write callback to STM
#We would do 256 (pagesize), but currently max number of bytes in callbacks 
#in firmware doesn't allow for this
ADDR_PER_WR_CB = 128

#Maximum number of flash callbacks to have pending in STM
#in order to (1) keep commands queued up for speed while not
#overflowing STM's UART RX buffer, and (2) to keep track of
#where we are on the PC side
#  erase_sector : each sector erase is one callback
#  write : each write of ADDR_PER_WR_CB bytes or less is one callback
#  read : each read of ADDR_PER_RD_CB bytes or less is one callback
#         (can read arbitrary length of flash with just
#          a single read command sent to STM, so only (2)
#          applies to read)
PENDING_CB_LIMIT = 10

def roundup_multiple(x, multiple):
  return x if x % multiple == 0 else x + multiple - x % multiple

def rounddown_multiple(x, multiple):
  return x if x % multiple == 0 else x - x % multiple

class M25_flash(Thread):
  _flash_ready = Event()
  _command_queue = []
  _wants_to_exit = False
  _commands_sent = 0
  _done_callbacks_received = 0
  _read_callbacks_received = 0
  _rd_cb_addrs = []
  _rd_cb_lens = []
  _rd_cb_data = []

  def __init__(self, link):
    super(M25_flash, self).__init__()
    self._flash_ready.set()
    self.link = link
    self.link.add_callback(0xF0, self._flash_done_callback)
    self.link.add_callback(0xF1, self._flash_read_callback)

  def flash_operations_left(self):
    return len(self._command_queue) + self._commands_sent - self._done_callbacks_received - self._read_callbacks_received

  def _flash_done_callback(self, data):
    self._done_callbacks_received += 1
    assert self._commands_sent - self._done_callbacks_received >= 0, "More done callbacks received than commands sent"

  def _flash_read_callback(self, data):
    #3 bytes addr, 1 byte length, length bytes data
    #append \x00 to the left side as it is a 3-byte big endian unsigned int and 
    #we unpack as a 4-byte big endian unsigned int
    self._read_callbacks_received += 1
    self._rd_cb_addrs.append(struct.unpack('>I','\x00' + data[0:3])[0])
    self._rd_cb_lens.append(struct.unpack('B',data[3])[0])
    self._rd_cb_data += list(struct.unpack(str(self._rd_cb_lens[-1]) + 'B',data[4:]))

  def _schedule_command(self, cmd, addr):
    self._command_queue.append((cmd, addr))

  def stop(self):
    self._wants_to_exit = True

  def run(self):
    while not self._wants_to_exit:
      while self.flash_operations_left() < 0:
        raise Exception('self.flash_operations_left() returns less than 0')
      if (len(self._command_queue) > 0) and ((self._commands_sent - self._done_callbacks_received - self._read_callbacks_received) < PENDING_CB_LIMIT):
        cmd, args = self._command_queue[0]
        self._command_queue = self._command_queue[1:]
        cmd_func = getattr(self, cmd)
        if cmd_func:
          cmd_func(*args)
      else:
        time.sleep(0.01)

  #Check that we received continuous addresses from the 
  #beginning of the flash read to the end, and that this
  #matches the length of the received data from those addrs
  def read_cb_sanity_check(self):
    expected_addrs = [self._rd_cb_addrs[0]]
    for length in self._rd_cb_lens[0:-1]:
      expected_addrs.append(expected_addrs[-1] + length)
    if self._rd_cb_addrs != expected_addrs:
      raise Exception('Addresses returned in read callback appear discontinuous')
    if sum(self._rd_cb_lens) != len(self._rd_cb_data):
      raise Exception('Length of read data does not match read callback lengths')

  def read(self, addr, length):
    self._schedule_command('_read', (addr, length))
  def _read(self, addr, length):
    msg_buf = struct.pack("<II", addr, length)
    self._commands_sent += int(ceil(float(length)/ADDR_PER_RD_CB))
    self.link.send_message(0xF1, msg_buf)

  def erase_sector(self, length):
    self._schedule_command('_erase_sector', (length,))
  def _erase_sector(self, addr):
    msg_buf = struct.pack("<I", addr)
    self._commands_sent += 1
    self.link.send_message(0xF2, msg_buf)

  def write(self, addr, data):
    self._schedule_command('_write', (addr,data))
  def _write(self, addr, data):
    while len(data) > ADDR_PER_WR_CB:
      data_to_send = data[:ADDR_PER_WR_CB]
      msg_header = struct.pack("<IB", addr, len(data_to_send))
      self._commands_sent += 1
      self.link.send_message(0xF0, msg_header+data_to_send)
      addr += ADDR_PER_WR_CB
      data = data[ADDR_PER_WR_CB:]
#    self._acquire_flash()
    msg_header = struct.pack("<IB", addr, len(data))
    self._commands_sent += 1
    self.link.send_message(0xF0, msg_header+data)

  def write_ihx(self, filename):
    ihx = IntelHex(filename)
    t1 = time.time()
    min_sector = rounddown_multiple(ihx.minaddr(), SECTORSIZE)
    max_sector = roundup_multiple(ihx.maxaddr(), SECTORSIZE)

    #Write hex file to flash
    print "Writing hex to flash..."
    for addr in range(min_sector, max_sector, SECTORSIZE):
      self.erase_sector(addr)
    min_page = rounddown_multiple(ihx.minaddr(), ADDR_PER_WR_CB)
    max_page = roundup_multiple(ihx.maxaddr(), ADDR_PER_WR_CB)
    for addr in range(min_page, max_page, ADDR_PER_WR_CB):
      self.write(addr, ihx.tobinstr(start=addr, size=ADDR_PER_WR_CB))
    ops_left = self.flash_operations_left()
    while ops_left != 0:
      sys.stdout.write("\r\033[K")
      print "\rFlash operations left =", ops_left,
      sys.stdout.flush()
      time.sleep(0.2)
      ops_left = self.flash_operations_left()
    sys.stdout.write("\r\033[K")
    print "\rFlash operations left =", ops_left
    sys.stdout.flush()
    t2 = time.time()
    print "Finished writing hex to flash, took %.2f seconds" % (t2 - t1)

    #Read bytes back from flash
    print "Reading hex back from flash..."
    self.read(0,ihx.maxaddr())
    ops_left = self.flash_operations_left()
    while ops_left != 0:
      sys.stdout.write("\r\033[K") #return to beginning of line and clear line
      print "Flash operations left =", ops_left,
      sys.stdout.flush()
      time.sleep(0.2)
      ops_left = self.flash_operations_left()
    sys.stdout.write("\r\033[K")
    print "Flash operations left =", ops_left
    sys.stdout.flush()
    t3 = time.time()
    print "Finished reading hex from flash, took %.2f seconds" % (t3 - t2)

    #Check that bytes read back from flash match hex file
    print "Verifying that bytes read back from flash match hex file..."
    self.read_cb_sanity_check()
    if self._rd_cb_data != list(ihx.tobinarray(start=ihx.minaddr(), size=ihx.maxaddr()-ihx.minaddr())):
      raise Exception('Data read from flash does not match configuration file')
    print "Data from flash matches hex file, update successful"

    #Finished, report execution time
    print "Total time was %.2f seconds" %(t3 - t1)

#Write the hex file 'swift-nap_mcs.mcs' to the flash
#on the device connected via serial port /dev/ttyUSB0
if __name__ == "__main__":
  import argparse
  import serial_link
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
  #Check that final address of the hex file is not in the last sector of the flash
  ihx = IntelHex(flash_file)
  print "Checking to make sure hex file's maximum address is not in last sector"
  print "We don't want to erase the device's authentication hash"
  print "  First address of flash's last sector =", hex(FLASHSIZE-SECTORSIZE)
  print "  Maximum address of hex file          =", hex(ihx.maxaddr())
  assert ihx.maxaddr() < (FLASHSIZE-SECTORSIZE), "Maximum address in hex file is in last sector"
  #Create SerialLink and Flash objects
  link = serial_link.SerialLink(port=serial_port)
  link.add_callback(serial_link.MSG_PRINT, serial_link.default_print_callback)
  piksi_flash = M25_flash(link)
  piksi_flash.start()
  #Write configuration file to FPGA's flash
  piksi_flash.write_ihx(flash_file)
  #Clean up before exiting
  piksi_flash.stop()
  link.close()
  sys.exit()

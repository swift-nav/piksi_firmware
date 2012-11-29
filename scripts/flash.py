#import wx
import struct
import time
from intelhex import IntelHex
from threading import Thread, Event
from math import ceil

PAGESIZE = 256
SECTORSIZE = 64*1024
FLASHSIZE = 1024*1024

#Maximum number of addresses returned by STM in a single read callback
#Defined in m25_flash.c in the read callback
ADDRS_PER_READ_CALLBACK = 16

#Maximum number of addresses written in a single write callback to STM
ADDRS_PER_WRITE_CALLBACK = 128

#Maximum number of flash callbacks to have pending in STM
#in order to (1) keep commands queued up for speed while not
#overflowing STM's UART RX buffer, and (2) to keep track of
#where we are on the PC side
#  erase_sector : each sector erase is one callback
#  write : each write of 128 bytes or less is one callback
#  read : each read of 16 bytes or less is one callback
#         (can read arbitrary length of flash with just
#          a single read callback to STM, so only (2)
#          applies to read)
PENDING_COMMANDS_LIMIT = 20

def roundup_multiple(x, multiple):
  return x if x % multiple == 0 else x + multiple - x % multiple

def rounddown_multiple(x, multiple):
  return x if x % multiple == 0 else x - x % multiple

class Flash(Thread):
  _flash_ready = Event()
  _command_queue = []
  _wants_to_exit = False
  _callbacks_expected = 0
  _done_callbacks_received = 0
  _read_callbacks_received = 0
  rd_cb_addrs = []
  rd_cb_lens = []
  rd_cb_data = []
  last_data = []

  def __init__(self, link):
    super(Flash, self).__init__()
    self._flash_ready.set()
    self.link = link
    self.link.add_callback(0xF0, self._flash_done_callback)
    self.link.add_callback(0xF1, self._flash_read_callback)

  def flash_callbacks_left(self):
    return len(self._command_queue) + self._callbacks_expected - self._done_callbacks_received - self._read_callbacks_received

  #called by STM after an erase_sector or a write
  def _flash_done_callback(self, data):
    self._flash_ready.set()
    self._done_callbacks_received += 1

  #called by STM after a read
  def _flash_read_callback(self, data):
    self._read_callbacks_received += 1
    #data = 3 bytes addr, 1 byte length, length bytes data
    addr = None
    length = None
    try:
      #append \x00 to the left side of the address as it is a 3-byte big endian
      #unsigned int and we unpack it as a 4-byte big endian unsigned int
      addr = struct.unpack('>I','\x00' + data[0:3])[0]
      self.rd_cb_addrs.append(addr)
#      self.rd_cb_addrs.append(struct.unpack('>I','\x00' + data[0:3])[0])
      length = struct.unpack('B',data[3])[0]
      self.rd_cb_lens.append(length)
#      self.rd_cb_lens.append(struct.unpack('B',data[3])[0])
      self.rd_cb_data += list(struct.unpack(str(self.rd_cb_lens[-1]) + 'B',data[4:]))
      if length != 16:
        print '  addr =', addr
        print '  length =', length
        print '  len of data =', len(data)
        print '  data =', [ord(i) for i in data]
        print '  last_data =', [ord(i) for i in self.last_data]
        print '  hex(addr) =', hex(addr)
        print '  flash operations left =', self.flash_callbacks_left()
#        sys.exit()
    except Exception:
      print '  addr =', addr
      print '  length =', length
      print '  len of data =', len(data)
      print '  data =', [ord(i) for i in data]
      print '  last_data =', [ord(i) for i in self.last_data]
      print '  hex(addr) =', hex(addr)
#      sys.exit()
    if addr % 0x1000 == 0:
      print '  addr =', addr
      print '  length =', length
      print '  len of data =', len(data)
      print '  data =', [ord(i) for i in data]
      print '  last_data =', [ord(i) for i in self.last_data]
      print '  hex(addr) =', hex(addr)
    self.last_data = data

  #Check that we received continuous addresses from the 
  #beginning of the flash read to the end, and that this
  #matches the length of the received data from those addrs
  def read_cb_sanity_check(self):
    expected_addrs = [self.rd_cb_addrs[0]]
    for length in self.rd_cb_lens[0:-1]:
      expected_addrs.append(expected_addrs[-1] + length)
    if self.rd_cb_addrs != expected_addrs:
      print self.rd_cb_addrs[0:10]
      print expected_addrs[0:10]
      raise Exception('Addresses returned in read callback appear discontinuous')
    if sum(self.rd_cb_lens) != len(self.rd_cb_data):
      raise Exception('Length of read data does not match read callback lengths')

  def _acquire_flash(self):
    self._flash_ready.wait()
    self._flash_ready.clear()

  def _schedule_command(self, cmd, addr):
    self._command_queue.append((cmd, addr))

  def stop(self):
    self._wants_to_exit = True

  def run(self):
    while not self._wants_to_exit:
      if self._command_queue and (self._callbacks_expected - self._done_callbacks_received - self._read_callbacks_received < PENDING_COMMANDS_LIMIT):
        cmd, args = self._command_queue[0]
        self._command_queue = self._command_queue[1:]
        cmd_func = getattr(self, cmd)
        if cmd_func:
          cmd_func(*args)
      else:
        time.sleep(0.001)

  def read(self, addr, length):
    self._schedule_command('_read', (addr, length))
  def _read(self, addr, length):
    self.rd_cb_addrs = []
    self.rd_cb_lens = []
    self.rd_cb_data = []
    msg_buf = struct.pack("<II", addr, length)
    self._callbacks_expected += int(ceil(float(length)/ADDRS_PER_READ_CALLBACK)) 
    self.link.send_message(0xF1, msg_buf)

  def erase_sector(self, length):
    self._schedule_command('_erase_sector', (length,))
  def _erase_sector(self, addr):
#    self._acquire_flash()
    msg_buf = struct.pack("<I", addr)
    self._callbacks_expected += 1
    self.link.send_message(0xF2, msg_buf)

  def write(self, addr, data):
    self._schedule_command('_write', (addr,data))
  def _write(self, addr, data):
    while len(data) > ADDRS_PER_WRITE_CALLBACK:
      data_to_send = data[:ADDRS_PER_WRITE_CALLBACK]
#      self._acquire_flash()
      msg_header = struct.pack("<IB", addr, len(data_to_send))
      self._callbacks_expected += 1
      self.link.send_message(0xF0, msg_header+data_to_send)
      addr += ADDRS_PER_WRITE_CALLBACK
      data = data[ADDRS_PER_WRITE_CALLBACK:]
#    self._acquire_flash()
    msg_header = struct.pack("<IB", addr, len(data))
    self._callbacks_expected += 1
    self.link.send_message(0xF0, msg_header+data)

#  def write_ihx(self, filename):
#    self._schedule_command('_write_ihx', (filename,))
  def write_ihx(self, filename):
    ihx = IntelHex(filename)
    min_sector = rounddown_multiple(ihx.minaddr(), SECTORSIZE)
    max_sector = roundup_multiple(ihx.maxaddr(), SECTORSIZE)
    for addr in range(min_sector, max_sector, SECTORSIZE):
      self.erase_sector(addr)
    min_page = rounddown_multiple(ihx.minaddr(), 128)
    max_page = roundup_multiple(ihx.maxaddr(), 128)
    for addr in range(min_page, max_page, 128):
      self.write(addr, ihx.tobinstr(start=addr, size=128))

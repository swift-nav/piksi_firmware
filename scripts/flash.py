#import wx
import struct
import time
from intelhex import IntelHex
from threading import Thread, Event
import sys
from math import ceil

PAGESIZE = 256
SECTORSIZE = 64*1024
FLASHSIZE = 1024*1024
#max number of flash operations to have pending in STM
PENDING_COMMANDS_LIMIT = 20

def roundup_multiple(x, multiple):
  return x if x % multiple == 0 else x + multiple - x % multiple

def rounddown_multiple(x, multiple):
  return x if x % multiple == 0 else x - x % multiple

class Flash(Thread):
  _flash_ready = Event()
  _command_queue = []
  _wants_to_exit = False
  _commands_sent = 0
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

  def flash_operations_left(self):
    return len(self._command_queue) + self._commands_sent - self._done_callbacks_received - self._read_callbacks_received

  def _flash_done_callback(self, data):
    self._flash_ready.set()
    self._done_callbacks_received += 1

  def _flash_read_callback(self, data):
    #3 bytes addr, 1 byte length, length bytes data
    #append \x00 to the left side as it is a 3-byte big endian unsigned int and 
    #we unpack as a 4-byte big endian unsigned int
    self._read_callbacks_received += 1
    addr = None
    length = None
    try:
      addr = struct.unpack('>I','\x00' + data[0:3])[0]
      self.rd_cb_addrs.append(addr)
      length = struct.unpack('B',data[3])[0]
      self.rd_cb_lens.append(length)
      self.rd_cb_data += list(struct.unpack(str(self.rd_cb_lens[-1]) + 'B',data[4:]))
#      self.rd_cb_data += list(struct.unpack('20B',data[4:]))
      if length != 16:
        print "read_callbacks_received = ", self._read_callbacks_received
        print '  addr =', addr
        print '  length =', length
        print '  len of data =', len(data)
        print '  data =', [ord(i) for i in data]
        print '  last_data =', [ord(i) for i in self.last_data]
        print '  hex(addr) =', hex(addr)
        print '  flash operations left =', self.flash_operations_left()
        sys.exit()
    except Exception:
      print "read_callbacks_received = ", self._read_callbacks_received
      print '  addr =', addr
      print '  length =', length
      print '  len of data =', len(data)
      print '  data =', [ord(i) for i in data]
      print '  last_data =', [ord(i) for i in self.last_data]
      print '  hex(addr) =', hex(addr)
      sys.exit()
    if self._read_callbacks_received % 0x1000 == 0:
      print "read_callbacks_received = ", self._read_callbacks_received
      print '  addr =', addr
      print '  length =', length
      print '  len of data =', len(data)
      print '  data =', [ord(i) for i in data]
      print '  last_data =', [ord(i) for i in self.last_data]
      print '  hex(addr) =', hex(addr)
    self.last_data = data

  def _acquire_flash(self):
    self._flash_ready.wait()
    self._flash_ready.clear()

  def _schedule_command(self, cmd, addr):
    self._command_queue.append((cmd, addr))

  def stop(self):
    self._wants_to_exit = True

  def run(self):
    while not self._wants_to_exit:
      if (len(self._command_queue) > 0) and ((self._commands_sent - self._done_callbacks_received - self._read_callbacks_received) < PENDING_COMMANDS_LIMIT):
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
    msg_buf = struct.pack("<II", addr, length)
    self._commands_sent += int(ceil(float(length)/16))
    self.link.send_message(0xF1, msg_buf)

  def erase_sector(self, length):
    self._schedule_command('_erase_sector', (length,))
  def _erase_sector(self, addr):
#    self._acquire_flash()
    msg_buf = struct.pack("<I", addr)
    self._commands_sent += 1
    self.link.send_message(0xF2, msg_buf)

  def write(self, addr, data):
    self._schedule_command('_write', (addr,data))
  def _write(self, addr, data):
    MAXSIZE = 128
    while len(data) > MAXSIZE:
      data_to_send = data[:MAXSIZE]
#      self._acquire_flash()
      msg_header = struct.pack("<IB", addr, len(data_to_send))
      self._commands_sent += 1
      self.link.send_message(0xF0, msg_header+data_to_send)
      addr += MAXSIZE
      data = data[MAXSIZE:]
#    self._acquire_flash()
    msg_header = struct.pack("<IB", addr, len(data))
    self._commands_sent += 1
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

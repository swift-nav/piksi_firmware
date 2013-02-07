import struct
import time
from intelhex import IntelHex
from threading import Thread, Event

PAGESIZE = 256
SECTORSIZE = 64*1024
FLASHSIZE = 1024*1024

def roundup_multiple(x, multiple):
  return x if x % multiple == 0 else x + multiple - x % multiple

def rounddown_multiple(x, multiple):
  return x if x % multiple == 0 else x - x % multiple

class Flash(Thread):
  _flash_ready = Event()
  _command_queue = []
  _wants_to_exit = False

  def __init__(self, link):
    super(Flash, self).__init__()
    self._flash_ready.set()
    self.link = link
    self.link.add_callback(0xF0, self._flash_done_callback)

  def _flash_done_callback(self, data):
    print "Flash done callback"
    self._flash_ready.set()

  def _acquire_flash(self):
    self._flash_ready.wait()
    self._flash_ready.clear()

  def _schedule_command(self, cmd, addr):
    self._command_queue.append((cmd, addr))

  def stop(self):
    self._wants_to_exit = True

  def run(self):
    while not self._wants_to_exit:
      if self._command_queue:
        cmd, args = self._command_queue[0]
        self._command_queue = self._command_queue[1:]
        cmd_func = getattr(self, cmd)
        if cmd_func:
          cmd_func(*args)
      else:
        time.sleep(0.1)

  def read(self, addr, length):
    self._schedule_command('_read', (addr, length))
  def _read(self, addr, length):
    msg_buf = struct.pack("<II", addr, length)
    self.link.send_message(0xF1, msg_buf)

  def erase_sector(self, length):
    self._schedule_command('_erase_sector', (length,))
  def _erase_sector(self, addr):
    self._acquire_flash()
    msg_buf = struct.pack("<I", addr)
    self.link.send_message(0xF2, msg_buf)

  def write(self, addr, data):
    self._schedule_command('_write', (addr,data))
  def _write(self, addr, data):
    MAXSIZE = 128
    while len(data) > MAXSIZE:
      data_to_send = data[:MAXSIZE]
      self._acquire_flash()
      msg_header = struct.pack("<IB", addr, len(data_to_send))
      self.link.send_message(0xF0, msg_header+data_to_send)
      addr += MAXSIZE
      data = data[MAXSIZE:]
    self._acquire_flash()
    msg_header = struct.pack("<IB", addr, len(data))
    self.link.send_message(0xF0, msg_header+data)

  def write_ihx(self, filename):
    self._schedule_command('_write_ihx', (filename,))
  def _write_ihx(self, filename):
    ihx = IntelHex(filename)
    
    print "Erasing."
    min_sector = rounddown_multiple(ihx.minaddr(), SECTORSIZE)
    max_sector = roundup_multiple(ihx.maxaddr(), SECTORSIZE)
    for addr in range(min_sector, max_sector, SECTORSIZE):
      self._erase_sector(addr)

    print "Writing."
    min_page = rounddown_multiple(ihx.minaddr(), PAGESIZE)
    max_page = roundup_multiple(ihx.maxaddr(), PAGESIZE)
    for addr in range(min_page, max_page, PAGESIZE):
      self._write(addr, ihx.tobinstr(start=addr, size=PAGESIZE))



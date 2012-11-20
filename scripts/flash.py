#import wx
import struct
import time
from intelhex import IntelHex
from threading import Thread, Event

PAGESIZE = 256
SECTORSIZE = 64*1024
FLASHSIZE = 1024*1024
#max number of flash operations to have pending in STM
PENDING_COMMANDS_LIMIT = 5

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

  def __init__(self, link):
    super(Flash, self).__init__()
    self._flash_ready.set()
    self.link = link
    self.link.add_callback(0xF0, self._flash_done_callback)

  def flash_operations_left(self):
    return len(self._command_queue) + self._commands_sent - self._done_callbacks_received

  def _flash_done_callback(self, data):
    self._flash_ready.set()
    self._done_callbacks_received += 1
    assert self._commands_sent - self._done_callbacks_received >= 0, "More callbacks received than commands sent"

  def _acquire_flash(self):
    self._flash_ready.wait()
    self._flash_ready.clear()

  def _schedule_command(self, cmd, addr):
    self._command_queue.append((cmd, addr))

  def stop(self):
    self._wants_to_exit = True

  def run(self):
    while not self._wants_to_exit:
      if self._command_queue and (self._commands_sent - self._done_callbacks_received < PENDING_COMMANDS_LIMIT):
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
    self._commands_sent += 1
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

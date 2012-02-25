import struct
import time
from intelhex import IntelHex

PAGESIZE = 256
SECTORSIZE = 64*1024
FLASHSIZE = 1024*1024

def roundup_multiple(x, multiple):
  return x if x % multiple == 0 else x + multiple - x % multiple

def rounddown_multiple(x, multiple):
  return x if x % multiple == 0 else x - x % multiple

class Flash:
  _flash_busy = False

  def __init__(self, link):
    self.link = link
    self.link.add_callback(0xF0, self._flash_done_callback)

  def _flash_done_callback(self, data):
    print "Flash done callback"
    self._flash_busy = False

  def wait_for_flash(self):
    while self._flash_busy == True:
      time.sleep(0.1)
    self._flash_busy = True

  def read(self, addr, length):
    msg_buf = struct.pack("<II", addr, length)
    self.link.send_message(0xF1, msg_buf)

  def erase_sector(self, addr):
    self.wait_for_flash()
    msg_buf = struct.pack("<I", addr)
    self.link.send_message(0xF2, msg_buf)

  def write(self, addr, data):
    MAXSIZE = 250
    while len(data) > MAXSIZE:
      data_to_send = data[:MAXSIZE]
      self.wait_for_flash()
      msg_header = struct.pack("<IB", addr, len(data_to_send))
      self.link.send_message(0xF0, msg_header+data_to_send)
      addr += MAXSIZE
      data = data[MAXSIZE:]
    self.wait_for_flash()
    msg_header = struct.pack("<IB", addr, len(data))
    self.link.send_message(0xF0, msg_header+data)

  def write_ihx(self, filename):
    ihx = IntelHex(filename)
    for addr in range(rounddown_multiple(ihx.minaddr(), SECTORSIZE), roundup_multiple(ihx.maxaddr(), SECTORSIZE)+1, PAGESIZE):
      if addr % SECTORSIZE == 0:
        self.erase_sector(addr)
      self.write(addr, ihx.tobinstr(start=addr, size=PAGESIZE))
      

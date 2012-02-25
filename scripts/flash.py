import struct

class Flash:

  def __init__(self, link):
    self.link = link

  def read(self, addr, length):
    msg_buf = struct.pack("<II", addr, length)
    self.link.send_message(0xF1, msg_buf)


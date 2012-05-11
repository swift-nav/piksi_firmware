from traits.api import Str, Instance, Dict, HasTraits, Array, Float, on_trait_change, Button, Int
from traitsui.api import Item, View

import threading
import time
import struct
from ctypes import create_string_buffer

from almanac import Almanac

MSG_ACQUISITION_SETUP = 0x69

#class BackgroundTimer(threading.Thread):
  #def __init__(self, callback):
    #super(BackgroundTimer, self).__init__()
    #self.cb = callback

  #def run(self):
    #while True:
      #time.sleep(1)
      #self.cb()

class AlmanacView(HasTraits):
  python_console_cmds = Dict()
  download_almanac = Button(label='Download Alamanc')
  warm_start = Button(label='Warm Start')
  alm = Instance(Almanac)
  alm_txt = Str

  traits_view = View(
    Item('alm_txt', label='PRNs visible'),
    Item('download_almanac'),
    Item('warm_start')
  )

  def _warm_start_fired(self):
    self.update_alamanc_view()

    acq_prn_t = struct.Struct('<hhB')
    buff = create_string_buffer(32*acq_prn_t.size)
    dopps = dict(self.alm.get_dopps())

    for prn in range(1, 33):
      state = 0 #ACQ_PRN_SKIP
      cf_min = cf_max = 0.0
      if prn in dopps.keys():
        state = 1 #ACQ_PRN_UNTRIED
        cf_min = round(dopps[prn] - 3500)
        cf_max = round(dopps[prn] + 3500)
      acq_prn_t.pack_into(buff, (prn-1)*acq_prn_t.size, cf_min, cf_max, state)

    self.link.send_message(0x69, buff.raw)

  def update_alamanc_view(self):
    prns, dopps = zip(*self.alm.get_dopps())
    self.alm_txt = str(prns)

  def _download_almanac_fired(self):
    self.alm.download_almanac()
    #self.timer.start()
    self.update_alamanc_view()

  def __init__(self, link):
    super(AlmanacView, self).__init__()

    self.link = link

    self.alm = Almanac()

    #self.timer = BackgroundTimer(self.update_alamanc_view)

    self.python_console_cmds = {
      'alm': self.alm
    }


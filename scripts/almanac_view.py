from enthought.traits.api import Str, Instance, Dict, HasTraits, Array, Float, on_trait_change, Button, Int
from enthought.traits.ui.api import Item, View

import threading
import time

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
    pass

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


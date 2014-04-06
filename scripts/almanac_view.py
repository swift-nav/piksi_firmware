# Copyright (C) 2011-2014 Swift Navigation Inc.
# Contact: Fergus Noble <fergus@swift-nav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

from traits.api import Str, Instance, Dict, HasTraits, Array, Float, on_trait_change, Button, Int
from traitsui.api import Item, View, HSplit, VSplit, VGroup

import threading
import time
import struct

import sbp_piksi as ids
from almanac import Almanac

class AlmanacView(HasTraits):
  python_console_cmds = Dict()
  download_almanac = Button(label='Download latest alamanc')
  load_almanac = Button(label='Load alamanc from file')
  send_alm = Button(label='Send almanac to Piksi')
  send_time = Button(label='Send time to Piksi')
  alm = Instance(Almanac)
  alm_txt = Str

  traits_view = View(
  	VGroup(
	    Item('alm_txt', label='PRNs visible'),
	    Item('download_almanac'),
	    Item('load_almanac'),
	    Item('send_alm'),
	    Item('send_time')
  	)
  )

  def _send_time_fired(self):
    gps_secs = time.time() - (315964800 - 16)
    gps_week = int(gps_secs / (7*24*3600))
    gps_tow = gps_secs % (7*24*3600)
    print gps_week, gps_tow
    buff = struct.pack("<dH", gps_tow, gps_week)
    self.link.send_message(ids.SET_TIME, buff)

  def _send_alm_fired(self):
    self.update_alamanc_view()

    for sat in self.alm.sats:
      self.link.send_message(ids.ALMANAC, sat.packed())

  def update_alamanc_view(self):
    #prns, dopps = zip(*self.alm.get_dopps())
    self.alm_txt = str(self.alm.get_dopps())

  def _download_almanac_fired(self):
    self.alm.download_almanac()
    self.update_alamanc_view()

  def _load_almanac_fired(self):
    self.alm.load_almanac_file('current.alm')
    self.update_alamanc_view()

  def __init__(self, link):
    super(AlmanacView, self).__init__()

    self.link = link

    self.alm = Almanac()

    self.python_console_cmds = {
      'alm': self.alm
    }


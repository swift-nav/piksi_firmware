#!/usr/bin/env python
# Copyright (C) 2011-2014 Swift Navigation Inc.
# Contact: Fergus Noble <fergus@swift-nav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

from traits.api import Instance, Dict, HasTraits, Array, Float, on_trait_change, List, Int, Button, Bool, Str
from traitsui.api import Item, View, HGroup, VGroup, ArrayEditor, HSplit, TabularEditor, VSplit
from traitsui.tabular_adapter import TabularAdapter
from chaco.api import ArrayPlotData, Plot
from chaco.tools.api import ZoomTool, PanTool
from enable.api import ComponentEditor
from enable.savage.trait_defs.ui.svg_button import SVGButton
from pyface.api import GUI

import struct
import math
import os
import numpy as np
import datetime

import sbp_piksi as ids

class SimpleAdapter(TabularAdapter):
    columns = [('PRN', 0), ('Pseudorange',  1), ('Carrier Phase',  2), ('SNR', 3)]

class Observation:
  def from_binary(self, data):
    soln = struct.unpack('<3d3d3d3d7ddddHBB', data)
    self.pos_llh = [soln[0]*(180/math.pi), soln[1]*(180/math.pi), soln[2]]
    self.pos_ecef = soln[3:6]
    self.vel_ned = soln[6:9]
    self.vel_ecef = soln[9:12]
    self.err_cov = soln[12:19]
    self.clock_offset = soln[19]
    self.clock_bias = soln[20]
    self.gps_tow = soln[21]
    self.gps_week = soln[22]
    self.soln_valid = soln[23]
    self.n_used = soln[24]


class ObservationView(HasTraits):
  python_console_cmds = Dict()

  _obs_table_list = List()
  obs = Dict()

  name = Str('Rover')

  recording = Bool(False)

  record_button = SVGButton(
    label='Record', tooltip='Record Raw Observations',
    toggle_tooltip='Stop Recording', toggle=True,
    filename=os.path.join(os.path.dirname(__file__), 'images', 'fontawesome', 'floppy-o.svg'),
    toggle_filename=os.path.join(os.path.dirname(__file__), 'images', 'fontawesome', 'stop.svg'),
    width=16, height=16
  )

  def trait_view(self, view):
    return View(
      HGroup(
        Item('_obs_table_list', style = 'readonly', editor = TabularEditor(adapter=SimpleAdapter()), show_label=False),
        VGroup(
          Item('record_button', show_label=False),
        ),
        label = self.name,
        show_border = True
      )
    )

  def _record_button_fired(self):
    self.recording = not self.recording
    if not self.recording:
      if self.rinex_file is not None:
        self.rinex_file.close()
      self.rinex_file = None

  def rinex_save(self):
    if self.recording:
      if self.rinex_file is None:
        # If the file is being opened for the first time, write the RINEX header
        self.rinex_file = open(self.name+self.t.strftime("-%Y%m%d-%H%M%S.obs"),  'w')
        header = """     2.11           OBSERVATION DATA    G (GPS)             RINEX VERSION / TYPE
pyNEX                                   %s UTC PGM / RUN BY / DATE
                                                            MARKER NAME
                                                            OBSERVER / AGENCY
                                                            REC # / TYPE / VERS
                                                            ANT # / TYPE
   808673.9171 -4086658.5368  4115497.9775                  APPROX POSITION XYZ
        0.0000        0.0000        0.0000                  ANTENNA: DELTA H/E/N
     1     0                                                WAVELENGTH FACT L1/2
     3    C1    L1    S1                                    # / TYPES OF OBSERV
%s%13.7f     GPS         TIME OF FIRST OBS
                                                            END OF HEADER
""" % (
            datetime.datetime.utcnow().strftime("%Y%m%d %H%M%S"),
            self.t.strftime("  %Y    %m    %d    %H    %M"), self.t.second + self.t.microsecond * 1e-6,
        )
        self.rinex_file.write(header)

      prns = list(self.obs.iterkeys())
      self.rinex_file.write("%s %10.7f  0 %2d" % (self.t.strftime(" %y %m %d %H %M"),
                                                  self.t.second + self.t.microsecond*1e-6,
                                                  len(prns)))
      while len(prns) > 0:
          prns_ = prns[:12]
          prns = prns[12:]
          for prn in prns_:
              self.rinex_file.write('G%2d' % (prn+1))
          self.rinex_file.write('   ' * (12 - len(prns_)))
          self.rinex_file.write('\n')

      for prn in list(self.obs.iterkeys()):
          # G    3 C1C L1C D1C
          self.rinex_file.write("%14.3f  " % self.obs[prn][0])
          self.rinex_file.write("%14.3f  " % self.obs[prn][1])
          self.rinex_file.write("%14.3f  \n" % self.obs[prn][2])

      self.rinex_file.flush()

  def update_obs(self):
    self._obs_table_list = [(prn + 1,) + obs for prn, obs in sorted(self.obs.items(), key=lambda x: x[0])]

  def obs_packed_callback(self, data, sender=None):
    if (sender is not None and
        (self.relay ^ (sender == 0))):
      return

    hdr_fmt = "<IHB"
    hdr_size = struct.calcsize(hdr_fmt)
    tow, wn, seq = struct.unpack(hdr_fmt, data[:hdr_size])

    tow = float(tow) / 1000.0

    total = seq >> 4
    count = seq & ((1 << 4) - 1)

    obs_fmt = '<IiBBHB'
    obs_size = struct.calcsize(obs_fmt)
    n_obs = (len(data) - hdr_size) / obs_size
    obs_data = data[hdr_size:]

    # Confirm this packet is good.
    # Assumes no out-of-order packets
    if (count == 0):
      self.gps_tow = tow;
      self.gps_week = wn;
      self.prev_obs_total = total
      self.prev_obs_count = 0;
      self.obs = {}

    elif (self.gps_tow            != tow    or
          self.gps_week           != wn     or
          self.prev_obs_count + 1 != count  or
          self.prev_obs_total     != total):
      print "We dropped a packet. Skipping this observation sequence"
      self.prev_obs_count = -1;
      return;

    else:
      self.prev_obs_count = count

    # Save this packet
    # See sbp_piksi.h for format
    for i in range(n_obs):
      P, Li, Lf, snr, lock, prn = struct.unpack(obs_fmt, obs_data[:obs_size])
      obs_data = obs_data[obs_size:]
      self.obs[prn] = (
        float(P) / 1e2,
        float(Li) + float(Lf) / (1<<8),
        float(snr) / 4)

    if (count == total - 1):
      self.t = datetime.datetime(1980, 1, 6) + \
               datetime.timedelta(weeks=self.gps_week) + \
               datetime.timedelta(seconds=self.gps_tow)

      self.update_obs()
      self.rinex_save()

    return

  def old_obs_callback(self, data, sender=None):
    print "Received deprecated observation messages. Please update your Piksi."
    if (sender is not None and
        (self.relay ^ (sender == 0))):
      return

    hdr_fmt = "<dH"
    hdr_size = struct.calcsize(hdr_fmt)
    tow, wn = struct.unpack("<dH", data[:hdr_size])
    self.gps_tow = tow
    self.gps_week = wn
    self.t = datetime.datetime(1980, 1, 6) + \
             datetime.timedelta(weeks=self.gps_week) + \
             datetime.timedelta(seconds=self.gps_tow)

    # Observation message format
    # double P;      /**< Pseudorange (m) */
    # double L;      /**< Carrier-phase (cycles) */
    # float snr;     /**< Signal-to-Noise ratio */
    # u8 prn;        /**< Satellite number. */
    obs_fmt = '<ddfB'

    obs_size = struct.calcsize(obs_fmt)
    n_obs = (len(data) - hdr_size) / obs_size
    obs_data = data[hdr_size:]

    self.obs = {}
    for i in range(n_obs):
      P, L, snr, prn = struct.unpack(obs_fmt, obs_data[:obs_size])
      obs_data = obs_data[obs_size:]
      self.obs[prn] = (P, L, snr)

    self.update_obs()
    self.rinex_save()

  def ephemeirs_callback(self, data, sender=None):
    print "GOT AN EPH BRO"

  def __init__(self, link, name='Rover', relay=False):
    super(ObservationView, self).__init__()

    self.obs_count = 0

    self.gps_tow = 0.0
    self.gps_week = 0

    self.relay = relay
    self.name = name

    self.rinex_file = None

    self.link = link
    self.link.add_callback(ids.OLD_OBS, self.old_obs_callback)
    self.link.add_callback(ids.PACKED_OBS, self.obs_packed_callback)
    self.link.add_callback(ids.EPHEMERIS, self.ephemeris_callback)

    self.python_console_cmds = {
      'obs': self
    }


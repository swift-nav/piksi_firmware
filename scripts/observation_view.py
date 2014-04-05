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

from traits.api import Instance, Dict, HasTraits, Array, Float, on_trait_change, List, Int, Button, Bool
from traitsui.api import Item, View, HGroup, VGroup, ArrayEditor, HSplit, TabularEditor
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

  recording = Bool(False)

  record_button = SVGButton(
    label='', tooltip='Pause', toggle_tooltip='Run', toggle=True,
    filename=os.path.join(os.path.dirname(__file__), 'images', 'pause.svg'),
    toggle_filename=os.path.join(os.path.dirname(__file__), 'images', 'play.svg'),
    width=16, height=16
  )

  traits_view = View(
  	HSplit(
	    VGroup(
	      Item('_obs_table_list', style = 'readonly', editor = TabularEditor(adapter=SimpleAdapter()), show_label=False),
	      HGroup(
	        Item('record_button', show_label=False),
	      )
	    )
  	),
  )


  def _record_button_fired(self):
    self.recording = not self.recording
    if not self.recording:
      if self.rinex_file is not None:
        self.rinex_file.close()
      self.rinex_file = None

  def update_obs(self):
    self._obs_table_list = [(prn + 1,) + obs for prn, obs in sorted(self.obs.items(), key=lambda x: x[0])]

  def obs_callback(self, data):
    if self.rinex_file is None and self.recording:
      self.rinex_file = open(self.t.strftime("%Y%m%d-%H%M%S.obs"),  'w')
      header = """     2.11           OBSERVATION DATA    G (GPS)             RINEX VERSION / TYPE
pyNEX                                   %s UTC PGM / RUN BY / DATE 
                                                            MARKER NAME         
                                                            OBSERVER / AGENCY   
                                                            REC # / TYPE / VERS 
                                                            ANT # / TYPE        
   808673.9171 -4086658.5368  4115497.9775                  APPROX POSITION XYZ 
        0.0000        0.0000        0.0000                  ANTENNA: DELTA H/E/N
     1     0                                                WAVELENGTH FACT L1/2
     4    C1    L1    S1                                    # / TYPES OF OBSERV 
%s%13.7f     GPS         TIME OF FIRST OBS   
                                                            END OF HEADER       
""" % (
          datetime.datetime.utcnow().strftime("%Y%m%d %H%M%S"),
          self.t.strftime("  %Y    %m    %d    %H    %M"), self.t.second + self.t.microsecond * 1e-6,
      )
      self.rinex_file.write(header)
      self.rinex_file.flush()


    hdr_fmt = "<dH"
    hdr_size = struct.calcsize(hdr_fmt)
    tow, wn = struct.unpack("<dH", data[:hdr_size])
    self.gps_tow = tow
    self.gps_week = wn
    self.t = datetime.datetime(1980, 1, 5) + \
             datetime.timedelta(weeks=self.gps_week) + \
             datetime.timedelta(seconds=self.gps_tow)

    obs_fmt = '<ddfB'
    """
  double P;      /**< Pseudorange (m) */
  double L;      /**< Carrier-phase (cycles) */
  float snr;     /**< Signal-to-Noise ratio */
  u8 prn;        /**< Satellite number. */
    """

    obs_size = struct.calcsize(obs_fmt)
    self.n_obs = (len(data) - hdr_size) / obs_size
    obs_data = data[hdr_size:]

    self.obs = {}
    for i in range(self.n_obs):
      P, L, snr, prn = struct.unpack(obs_fmt, obs_data[:obs_size])
      obs_data = obs_data[obs_size:]
      self.obs[prn] = (P, L, snr)

    if self.recording:
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

    self.update_obs()

  def __init__(self, link):
    super(ObservationView, self).__init__()

    self.obs_count = 0
    self.n_obs = 1

    self.rinex_file = None

    self.link = link
    self.link.add_callback(ids.NEW_OBS, self.obs_callback)

    self.python_console_cmds = {
      'obs': self
    }


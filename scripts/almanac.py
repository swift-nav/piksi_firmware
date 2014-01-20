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

import math as m
import numpy as n
import time
import urllib2

NAV_GM = 3.986005e14
NAV_OMEGAE_DOT = 7.2921151467e-005
GPS_L1_HZ = 1.57542e9
NAV_C = 299792458.0

RANDOM_SUNDAY = 1283644784
WPR = (-2712219.0, -4316338.0, 3820996.0)

def time_of_week():
  return (time.time() - RANDOM_SUNDAY) % (7*24*60*60)

class Sat:
  def __init__(self, yuma_block):
    fields = map(lambda x: x[25:], yuma_block)
    self.prn     = int(fields[0])
    self.healthy = (int(fields[1]) == 0)
    self.ecc     = float(fields[2])
    self.toa     = float(fields[3])
    self.inc     = float(fields[4])
    self.rora    = float(fields[5])
    self.a       = float(fields[6])**2
    self.raaw    = float(fields[7])
    self.argp    = float(fields[8])
    self.ma      = float(fields[9])
    self.af0     = float(fields[10])
    self.af1     = float(fields[11])
    self.week    = int(fields[12])

  def calc_vis_dopp(self, time_of_week, receiver_pos, elevation_mask=10.0):
    tdiff = time_of_week - self.toa

    if tdiff > 302400.0:
      tdiff -= 604800.0
    elif tdiff < -302400.0:
      tdiff += 604800.0

    ma_dot = m.sqrt(NAV_GM / self.a**3)
    ma = self.ma + ma_dot*tdiff

    # Iteratively solve for the Eccentric Anomaly (from Keith Alter and David Johnston)
    ea = ma # Starting value for E
    ea_old = ea + 1

    while (abs (ea - ea_old) > 1.0E-14):
      ea_old = ea
      tempd1 = 1.0 - self.ecc * m.cos(ea_old)
      ea = ea + (ma - ea_old + self.ecc * m.sin(ea_old)) / tempd1
    ea_dot = ma_dot / tempd1

    # Begin calc for True Analomay and Argument of Latitude
    tempd2 = m.sqrt(1.0 - self.ecc * self.ecc)
    # [rad] Argument of Latitude = True Anomaly + Argument of Perigee
    al = m.atan2(tempd2 * m.sin(ea), m.cos(ea) - self.ecc) + self.argp
    al_dot = tempd2 * ea_dot / tempd1

    # Calculate corrected radius based on argument of latitude
    r = self.a * tempd1
    r_dot = self.a * self.ecc * m.sin(ea) * ea_dot

    # Calculate inclination based on argument of latitude
    inc = self.inc

    # Calculate position and velocity in orbital plane
    x = r * m.cos(al)
    y = r * m.sin(al)
    x_dot = r_dot * m.cos(al) - y * al_dot
    y_dot = r_dot * m.sin(al) + x * al_dot

    # Corrected longitude of ascending node
    om_dot = self.rora - NAV_OMEGAE_DOT
    om = self.raaw + tdiff * om_dot - NAV_OMEGAE_DOT * self.toa

    sat_pos = n.array([0.0, 0.0, 0.0])
    sat_vel = n.array([0.0, 0.0, 0.0])

    # Compute the satellite's position in Earth-Centered Earth-Fixed coordiates
    sat_pos[0] = x * m.cos(om) - y * m.cos(inc) * m.sin(om)
    sat_pos[1] = x * m.sin(om) + y * m.cos(inc) * m.cos(om)
    sat_pos[2] = y * m.sin(inc)

    tempd3 = y_dot * m.cos(inc)

    # Compute the satellite's velocity in Earth-Centered Eart-Fixed coordiates
    sat_vel[0] = -om_dot * sat_pos[1] + x_dot * m.cos(om) - tempd3 * m.sin(om)
    sat_vel[1] = om_dot * sat_pos[0] + x_dot * m.sin(om) + tempd3 * m.cos(om)
    sat_vel[2] = y_dot * m.sin(inc)

    # Compute the angle that a satellite makes with the horizon for
    # the passed receiver location
    vec_rec_sat = sat_pos - receiver_pos
    vec_rec_sat /= m.sqrt(n.vdot(vec_rec_sat, vec_rec_sat))

    # normalize the vectors to find the angle in between them
    angle_to_normal = m.acos(n.vdot(vec_rec_sat, receiver_pos)/m.sqrt(n.vdot(receiver_pos, receiver_pos)))
    angle_to_horizon = m.pi/2 - angle_to_normal
    # velocity of the satellite in the axis of the vector from receiver to satellite
    radial_velocity = n.vdot(vec_rec_sat, sat_vel)
    doppler_shift = GPS_L1_HZ * -radial_velocity / NAV_C

    if (angle_to_horizon > elevation_mask*(m.pi/180)) and (self.healthy):
      return (doppler_shift, angle_to_horizon)
    else:
      return (None, None)

  def packed(self):
    import struct
    return struct.pack("<ddddddddddHBBB",
      self.ecc, self.toa, self.inc, self.rora, self.a, self.raaw, self.argp,
      self.ma, self.af0, self.af1, self.week, self.prn, self.healthy, 1
    )

  def __str__(self):
    dopp, _ = self.calc_vis_dopp(time_of_week(), WPR)
    if dopp != None:
      return "PRN%02d\t@ %+7.1f Hz\n" % (self.prn, dopp)
    else:
      return "PRN%02d Not Visible\n" % self.prn

class Almanac:
  sats = None

  def almanac_valid(self):
    return (self.sats != None)

  def download_almanac(self):
    u = urllib2.urlopen('http://www.navcen.uscg.gov/?pageName=currentAlmanac&format=yuma')
    if u:
      self.process_yuma(u.readlines())

  def load_almanac_file(self, filename):
    fp = open(filename, 'r')
    if fp:
      self.process_yuma(fp.readlines())
    fp.close()

  def process_yuma(self, yuma):
    if yuma:
      blocks = []
      for (n, line) in enumerate(yuma):
        if line[:3] == "ID:":
          blocks += [yuma[n:n+13]]
      self.sats = map(lambda bl: Sat(bl), blocks)
    else:
      self.sats = None

  def get_dopps(self, tow=None, location=WPR):
    if not tow:
      tow = time_of_week()

    if self.sats:
      dopps = map(lambda s: (s.prn,)+s.calc_vis_dopp(tow, location, elevation_mask=0.0), self.sats)
      dopps = filter(lambda (prn, dopp, el): (dopp != None), dopps)
      return dopps
    else:
      return None

if __name__ == "__main__":
  alm = Almanac()
  print "Downloading current almanac"
  alm.download_almanac()
  print "Dopplers:"
  print alm.get_dopps()

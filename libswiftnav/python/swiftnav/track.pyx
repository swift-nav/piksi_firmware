# cython: profile=True

import numpy as np
cimport numpy as np
cimport libc.math
cimport cython
cimport track_c
cimport prns_c
from nav_msg cimport NavMsg
from nav_msg import NavMsg
from ephemeris_c cimport ephemeris_t
from track_c cimport channel_measurement_t, navigation_measurement_t
from common cimport *
from libc.stdlib cimport malloc, free
from libc.math cimport M_PI

@cython.boundscheck(False)
def track_correlate_cy(np.ndarray[char, ndim=1, mode="c"] rawSignal,
                       double codeFreq,
                       double remCodePhase,
                       double carrFreq,
                       double remCarrPhase,
                       np.ndarray[long, ndim=1, mode="c"] caCode,
                       settings):
      cdef double codePhaseStep = codeFreq/settings.samplingFreq
      cdef double carrPhaseStep = carrFreq * 2.0 * M_PI / settings.samplingFreq
      cdef unsigned int blksize = <unsigned int>libc.math.ceil((settings.codeLength - remCodePhase)/codePhaseStep)

      cdef double codePhase = remCodePhase
      cdef double carrPhase = remCarrPhase

      # A cunning trick
      cdef double carrSin = libc.math.sin(remCarrPhase)
      cdef double carrCos = libc.math.cos(remCarrPhase)
      cdef double carrSin_, carrCos_
      cdef double sinDelta = libc.math.sin(carrPhaseStep)
      cdef double cosDelta = libc.math.cos(carrPhaseStep)
      cdef double imag

      cdef double I_E, Q_E, I_P, Q_P, I_L, Q_L
      I_E = Q_E = I_P = Q_P = I_L = Q_L = 0

      cdef double earlyCode, promptCode, lateCode
      cdef double qBasebandSignal, iBasebandSignal

      cdef unsigned int i
      for i in range(blksize):
        earlyCode = caCode[<unsigned int>(libc.math.ceil(codePhase-0.5))]
        promptCode = caCode[<unsigned int>(libc.math.ceil(codePhase))]
        lateCode = caCode[<unsigned int>(libc.math.ceil(codePhase+0.5))]
        #earlyCode = caCode[<unsigned int>(codePhase-0.5)]
        #promptCode = caCode[<unsigned int>(codePhase)]
        #lateCode = caCode[<unsigned int>(codePhase+0.5)]

        #Mix signals to baseband
        #qBasebandSignal = libc.math.cos(carrPhase)*rawSignal[i]
        #iBasebandSignal = libc.math.sin(carrPhase)*rawSignal[i]
        qBasebandSignal = carrCos*rawSignal[i]
        iBasebandSignal = carrSin*rawSignal[i]
        # Cunning trick - rotate unit vector by an angle carrPhaseStep (delta)
        carrSin_ = carrSin*cosDelta + carrCos*sinDelta
        carrCos_ = carrCos*cosDelta - carrSin*sinDelta
        # This is unstable, need to normalise
        #imag = 1.0 / libc.math.sqrt(carrSin_*carrSin_ + carrCos_*carrCos_)
        imag = (3.0 - carrSin_*carrSin_ - carrCos_*carrCos_) / 2.0
        carrSin = carrSin_ * imag
        carrCos = carrCos_ * imag

        #Get early, prompt, and late I/Q correlations
        I_E += earlyCode * iBasebandSignal
        Q_E += earlyCode * qBasebandSignal
        I_P += promptCode * iBasebandSignal
        Q_P += promptCode * qBasebandSignal
        I_L += lateCode * iBasebandSignal
        Q_L += lateCode * qBasebandSignal

        codePhase += codePhaseStep
        carrPhase += carrPhaseStep
      remCodePhase = codePhase - 1023
      remCarrPhase = carrPhase % (2.0*M_PI)

      return (I_E, Q_E, I_P, Q_P, I_L, Q_L, blksize, remCodePhase, remCarrPhase)


def track_correlate(np.ndarray[char, ndim=1, mode="c"] rawSignal,
                       codeFreq, remCodePhase, carrFreq, remCarrPhase,
                       np.ndarray[char, ndim=1, mode="c"] caCode,
                       settings):
  cdef double init_code_phase = remCodePhase
  cdef double init_carr_phase = remCarrPhase
  cdef double I_E, Q_E, I_P, Q_P, I_L, Q_L
  cdef unsigned int blksize
  track_c.track_correlate(<s8*>&rawSignal[0], <s8*>&caCode[0],
                          &init_code_phase, codeFreq/settings.samplingFreq,
                          &init_carr_phase, carrFreq * 2.0 * M_PI / settings.samplingFreq,
                          &I_E, &Q_E, &I_P, &Q_P, &I_L, &Q_L, &blksize)
  return (I_E, Q_E, I_P, Q_P, I_L, Q_L, blksize, init_code_phase, init_carr_phase)

cdef class ChannelMeasurement:
  cdef channel_measurement_t meas

  def __cinit__(self, prn, code_phase, code_freq, carr_phase, carr_freq, TOW_ms, rx_time, snr):
    self.meas.prn = prn
    self.meas.code_phase_chips = code_phase
    self.meas.code_phase_rate = code_freq
    self.meas.carrier_phase = carr_phase
    self.meas.carrier_freq = carr_freq
    self.meas.time_of_week_ms = TOW_ms
    self.meas.receiver_time = rx_time
    self.meas.snr = snr

  def __repr__(self):
    #return "<ChannelMeasurement PRN %d>" % self.meas.prn
    return str((self.meas.prn,
                self.meas.code_phase_chips,
                self.meas.code_phase_rate,
                self.meas.carrier_phase,
                self.meas.carrier_freq,
                self.meas.time_of_week_ms,
                self.meas.receiver_time,
                self.meas.snr))


cdef class NavigationMeasurement:
  def __cinit__(self, pr, prr, TOT, sat_pos, sat_vel):
    self.meas.pseudorange = pr
    self.meas.pseudorange_rate = prr
    self.meas.TOT = TOT
    for i in range(3):
      self.meas.sat_pos[i] = sat_pos[i]
      self.meas.sat_vel[i] = sat_vel[i]

  def __repr__(self):
    return str((self.meas.pseudorange,
                self.meas.pseudorange_rate,
                self.meas.TOT,
                (self.meas.sat_pos[0], self.meas.sat_pos[1], self.meas.sat_pos[2]),
                (self.meas.sat_vel[0], self.meas.sat_vel[1], self.meas.sat_vel[2])))

def calc_navigation_measurement(double t, chan_meas, es):
  n_channels = len(chan_meas)
  nav_meas = [NavigationMeasurement(0, 0, 0, (0,0,0), (0,0,0)) for n in range(n_channels)]

  cdef channel_measurement_t** chan_meas_ptrs = <channel_measurement_t**>malloc(n_channels*sizeof(channel_measurement_t*))
  cdef navigation_measurement_t** nav_meas_ptrs = <navigation_measurement_t**>malloc(n_channels*sizeof(navigation_measurement_t*))
  cdef ephemeris_t** es_ptrs = <ephemeris_t**>malloc(n_channels*sizeof(ephemeris_t*))

  for n in range(n_channels):
    chan_meas_ptrs[n] = &((<ChannelMeasurement?>chan_meas[n]).meas)
    nav_meas_ptrs[n] = &((<NavigationMeasurement?>nav_meas[n]).meas)
    es_ptrs[n] = &((<NavMsg?>es[n]).eph)

  track_c.calc_navigation_measurement_(n_channels, chan_meas_ptrs, nav_meas_ptrs, t, es_ptrs)

  free(chan_meas_ptrs)
  free(nav_meas_ptrs)
  free(es_ptrs)

  return nav_meas


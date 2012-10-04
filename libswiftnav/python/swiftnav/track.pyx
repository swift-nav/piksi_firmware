# cython: profile=True

import numpy as np
cimport numpy as np
cimport libc.math
cimport cython
cimport track_c
cimport prns_c

cdef double PI = 3.1415926535897932384626433832795028841971693993751058209

@cython.boundscheck(False)
def track_correlate_cy(np.ndarray[char, ndim=1, mode="c"] rawSignal,
                       double codeFreq,
                       double remCodePhase,
                       double carrFreq,
                       double remCarrPhase,
                       np.ndarray[long, ndim=1, mode="c"] caCode,
                       settings):
      cdef double codePhaseStep = codeFreq/settings.samplingFreq
      cdef double carrPhaseStep = carrFreq * 2.0 * PI / settings.samplingFreq
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
      remCarrPhase = carrPhase % (2.0*PI)

      return (I_E, Q_E, I_P, Q_P, I_L, Q_L, blksize, remCodePhase, remCarrPhase)


def track_correlate(np.ndarray[char, ndim=1, mode="c"] rawSignal,
                       codeFreq, remCodePhase, carrFreq, remCarrPhase,
                       np.ndarray[long, ndim=1, mode="c"] caCode,
                       settings):
  cdef double init_code_phase = remCodePhase
  cdef double init_carr_phase = remCarrPhase
  cdef double I_E, Q_E, I_P, Q_P, I_L, Q_L
  cdef unsigned int blksize
  track_c.track_correlate(&rawSignal[0], &caCode[0],
                          &init_code_phase, codeFreq/settings.samplingFreq,
                          &init_carr_phase, carrFreq * 2.0 * PI / settings.samplingFreq,
                          &I_E, &Q_E, &I_P, &Q_P, &I_L, &Q_L, &blksize)
  return (I_E, Q_E, I_P, Q_P, I_L, Q_L, blksize, init_code_phase, init_carr_phase)

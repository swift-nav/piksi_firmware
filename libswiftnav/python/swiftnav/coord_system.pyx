import numpy as np
cimport numpy as np
cimport coord_system_c

def wgsllh2ecef(llh):
 cdef np.ndarray[np.double_t, ndim=1, mode="c"] llh_ = np.array(llh, dtype=np.double)
 cdef np.ndarray[np.double_t, ndim=1, mode="c"] ecef = np.empty(3, dtype=np.double)
 coord_system_c.wgsllh2ecef(&llh_[0], &ecef[0])
 return ecef

def wgsecef2llh(ecef):
 cdef np.ndarray[np.double_t, ndim=1, mode="c"] ecef_ = np.array(ecef, dtype=np.double)
 cdef np.ndarray[np.double_t, ndim=1, mode="c"] llh = np.empty(3, dtype=np.double)
 coord_system_c.wgsllh2ecef(&ecef_[0], &llh[0])
 return llh

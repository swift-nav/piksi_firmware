cdef extern from "coord_system.h":
 void wgsllh2ecef(double llh[3], double ecef[3])
 void wgsecef2llh(double ecef[3], double llh[3])


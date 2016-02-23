/*
 * Copyright (C) 2013-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_NMEA_H
#define SWIFTNAV_NMEA_H

#include <libswiftnav/common.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/time.h>

#include "track.h"

/** \addtogroup nmea
 * \{ */

#define NMEA_GGA_FIX_INVALID 0
#define NMEA_GGA_FIX_GPS     1
#define NMEA_GGA_FIX_DGPS    2
#define NMEA_GGA_FIX_PPS     3
#define NMEA_GGA_FIX_RTK     4
#define NMEA_GGA_FIX_FLOAT   5
#define NMEA_GGA_FIX_EST     6
#define NMEA_GGA_FIX_MANUAL  7
#define NMEA_GGA_FIX_SIM     8

#define MINUTES(X) fabs(60 * (R2D * (X) - ((s16)(R2D * (X)))));
#define MS2KNOTTS(x,y,z) sqrt((x)*(x) + (y)*(y) + (z)*(z)) * 1.94385
#define MS2KMHR(x,y,z) sqrt((x)*(x)+(y)*(y)+(z)*(z)) * (3600/1000)


void nmea_setup(void);
void nmea_gpgga(const double pos_llh[3], const gps_time_t *gps_t, u8 n_used,
                u8 fix_type, double hdop);
void nmea_gpgsa(const u8 *prns, u8 num_prns, const dops_t *dops);
void nmea_gpgsv(u8 n_used, const navigation_measurement_t *nav_meas,
                const gnss_solution *soln);
void nmea_gprmc(const gnss_solution *soln, const gps_time_t *gps_t);
void nmea_gpvtg(const gnss_solution *soln);
void nmea_gpgll(const gnss_solution *soln, const gps_time_t *gps_t);
void nmea_send_msgs(gnss_solution *soln, u8 n, 
                    navigation_measurement_t *nm);

/** Register a new dispatcher for NMEA messages
 *
 * \param send_func Pointer to dispatcher function.
 */
#define nmea_dispatcher_register(send_func) do {         \
  static struct nmea_dispatcher dispatcher = \
    { .send = send_func }; \
  _nmea_dispatcher_register(&dispatcher); \
} while(0)

/** \cond */
struct nmea_dispatcher {
  void (*send)(const char *msg, size_t msg_size);
  struct nmea_dispatcher *next;
};

void _nmea_dispatcher_register(struct nmea_dispatcher *);
/** \endcond */

/** \} */

#endif  /* SWIFTNAV_NMEA_H */


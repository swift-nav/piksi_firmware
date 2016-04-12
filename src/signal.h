/*
 * Copyright (C) 2011-2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 * Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SIGNAL_H
#define SWIFTNAV_SIGNAL_H

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>

/* Platform-specific code support ==== */
#define CODE_GPS_L1CA_SUPPORT     1
#define CODE_GPS_L2CM_SUPPORT     0
#define CODE_SBAS_L1CA_SUPPORT    1
#define CODE_GLO_L1CA_SUPPORT     0
#define CODE_GLO_L2CA_SUPPORT     0
/* =================================== */

/** \addtogroup signal
 * \{ */

/* Number of signals on each code supported
 * on the current hardware platform. */
#define PLATFORM_SIGNAL_COUNT_GPS_L1CA      (CODE_GPS_L1CA_SUPPORT ?          \
                                             NUM_SIGNALS_GPS_L1CA : 0)
#define PLATFORM_SIGNAL_COUNT_GPS_L2CM      (CODE_GPS_L2CM_SUPPORT ?          \
                                             NUM_SIGNALS_GPS_L2CM : 0)
#define PLATFORM_SIGNAL_COUNT_SBAS_L1CA     (CODE_SBAS_L1CA_SUPPORT ?         \
                                             NUM_SIGNALS_SBAS_L1CA : 0)
#define PLATFORM_SIGNAL_COUNT_GLO_L1CA      (CODE_GLO_L1CA_SUPPORT ?          \
                                             NUM_SIGNALS_GLO_L1CA : 0)
#define PLATFORM_SIGNAL_COUNT_GLO_L2CA      (CODE_GLO_L2CA_SUPPORT ?          \
                                             NUM_SIGNALS_GLO_L2CA : 0)

/* Number of signals on each constellation supported
 * on the current hardware platform. */
#define PLATFORM_SIGNAL_COUNT_GPS     (PLATFORM_SIGNAL_COUNT_GPS_L1CA +       \
                                       PLATFORM_SIGNAL_COUNT_GPS_L2CM)
#define PLATFORM_SIGNAL_COUNT_SBAS    (PLATFORM_SIGNAL_COUNT_SBAS_L1CA)
#define PLATFORM_SIGNAL_COUNT_GLO     (PLATFORM_SIGNAL_COUNT_GLO_L1CA +       \
                                       PLATFORM_SIGNAL_COUNT_GLO_L2CA)

/* Total number of signal supported
 * on the current hardware platform. */
#define PLATFORM_SIGNAL_COUNT         (PLATFORM_SIGNAL_COUNT_GPS +            \
                                       PLATFORM_SIGNAL_COUNT_SBAS +           \
                                       PLATFORM_SIGNAL_COUNT_GLO)

/* \} */

void signal_init(void);
gnss_signal_t sid_from_global_index(u16 global_index);
gnss_signal_t sid_from_constellation_index(enum constellation constellation,
                                           u16 constellation_index);
u16 sid_to_global_index(gnss_signal_t sid);
u16 sid_to_constellation_index(gnss_signal_t sid);
bool sid_supported(gnss_signal_t sid);
bool code_supported(enum code code);

#endif /* SWIFTNAV_SIGNAL_H */

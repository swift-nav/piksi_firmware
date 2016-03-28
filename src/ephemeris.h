/*
 * Copyright (C) 2015 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Gareth McMullin <gareth@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_EPHEMERIS_H
#define SWIFTNAV_EPHEMERIS_H

#include <libswiftnav/constants.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/signal.h>

void ephemeris_setup(void);
void ephemeris_lock(void);
void ephemeris_unlock(void);
void ephemeris_new(ephemeris_t *e);
void ephemeris_clear(gnss_signal_t sid);
ephemeris_t *ephemeris_get(gnss_signal_t sid);

#endif

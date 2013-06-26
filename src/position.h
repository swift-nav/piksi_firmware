/*
 * Copyright (C) 2013 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_POSITION_H
#define SWIFTNAV_POSITION_H

#include <libswiftnav/common.h>
#include <libswiftnav/pvt.h>

/** \addtogroup position
 * \{ */

typedef enum {
  POSITION_UNKNOWN = 0,
  POSITION_GUESS,
  POSITION_STATIC,
  POSITION_FIX,
} position_quality_t;

/** \} */

extern position_quality_t position_quality;
extern gnss_solution position_solution;

void position_setup(void);
void position_updated(void);

#endif  /* SWIFTNAV_POSITION_H */

